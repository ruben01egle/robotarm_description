// Cartesian jogging of the arm without ros2_control, to try the kinematics plugin in RViz.
//
// Takes the place of joint_state_publisher_gui in the display setup:
//
//   cmd_vel (Twist)  ->  this node  ->  /joint_states  ->  robot_state_publisher  ->  TF  ->  RViz
//
// Every cycle the twist is turned into a Cartesian step dx = twist * dt, the kinematics plugin
// converts it into a joint step (damped inverse Jacobian), the step is limited to the joint limits
// of the URDF (JointLimiter.hpp), added to the joint positions and published.
//
// The twist is expressed in the BASE frame of the robot: linear [m/s] and angular [rad/s] velocity
// of the tcp. Nothing is simulated except the kinematics: no controller, no dynamics.

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "JointLimiter.hpp"
#include "robotarm_kinematics/KinematicsCore.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using robotarm_kinematics::JointLimit;
using robotarm_kinematics::StepLimit;
using Vector6 = Eigen::Matrix<double, 6, 1>;

constexpr int kLogThrottleMs = 1000;

class CartesianJogNode : public rclcpp::Node
{
public:
    CartesianJogNode()
    : rclcpp::Node("cartesian_jog")
    {
        const std::string robot_description = declare_parameter<std::string>("robot_description", "");
        const std::string plugin = declare_parameter<std::string>(
            "kinematics_plugin", "robotarm_kinematics/KinematicsCore");
        const double rate = declare_parameter<double>("rate", 100.0);
        // the last twist keeps being applied until it is this old: so this is also how far the tool
        // coasts after the last message. Keep it a few publish periods.
        twist_timeout_ = declare_parameter<double>("twist_timeout", 0.2);
        const std::vector<double> initial = declare_parameter<std::vector<double>>(
            "initial_joint_positions", std::vector<double>{0.0, -1.57, 1.57, 0.0, 0.0, 0.0});

        if (robot_description.empty()) {
            throw std::runtime_error("parameter 'robot_description' is empty");
        }
        if (!std::isfinite(rate) || rate <= 0.0) {
            throw std::runtime_error("parameter 'rate' must be > 0");
        }
        if (!std::isfinite(twist_timeout_) || twist_timeout_ <= 0.0) {
            throw std::runtime_error("parameter 'twist_timeout' must be > 0");
        }
        period_ = 1.0 / rate;

        load_kinematics(plugin, robot_description);
        read_robot_model();
        set_initial_pose(initial);

        const auto n = static_cast<Eigen::Index>(joint_names_.size());
        dq_ = Eigen::VectorXd::Zero(n);
        twist_.setZero();

        joint_state_.name = joint_names_;
        joint_state_.position.assign(joint_names_.size(), 0.0);
        joint_state_.velocity.assign(joint_names_.size(), 0.0);

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {on_twist(*msg);});

        last_tick_ = Clock::now();
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_)),
            [this]() {tick();});

        RCLCPP_INFO(
            get_logger(),
            "Jogging %zu joints, tcp link '%s', %.0f Hz. Send base frame twists to '%s' "
            "(stops %.2f s after the last one).",
            joint_names_.size(), tcp_link_.c_str(), rate, twist_sub_->get_topic_name(), twist_timeout_);
    }

private:
    // loaded through pluginlib, the way a ros2_control controller loads it. Only the interface is used.
    void load_kinematics(const std::string & plugin, const std::string & robot_description)
    {
        loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
            "kinematics_interface", "kinematics_interface::KinematicsInterface");
        kinematics_ = loader_->createUniqueInstance(plugin);
        if (!kinematics_->initialize(robot_description, get_node_parameters_interface(), "")) {
            throw std::runtime_error("kinematics plugin '" + plugin + "' failed to initialize");
        }
    }

    // Joint names (chain order, the order of q_), joint limits and tcp link come from the plugin, which
    // parsed and validated the URDF in initialize(). That is not part of kinematics_interface, so the
    // plugin has to be a KinematicsCore (or derived from it).
    void read_robot_model()
    {
        auto * core = dynamic_cast<robotarm_kinematics::KinematicsCore *>(kinematics_.get());
        if (!core) {
            throw std::runtime_error("the kinematics plugin is not a robotarm_kinematics::KinematicsCore");
        }
        std::vector<robotarm_kinematics::KinematicsCore::Limits> limits;
        if (!core->get_joint_names(joint_names_) || !core->get_joint_limits(limits) ||
            !core->get_tcp_link_name(tcp_link_))
        {
            throw std::runtime_error("the kinematics plugin does not provide the robot model");
        }
        if (joint_names_.empty() || limits.size() != joint_names_.size()) {
            throw std::runtime_error("the kinematics plugin returned inconsistent joint names and limits");
        }
        limits_.clear();
        for (const auto & l : limits) {
            limits_.push_back({l.min, l.max, l.velocity});
        }
    }

    void set_initial_pose(const std::vector<double> & initial)
    {
        if (initial.size() != joint_names_.size()) {
            throw std::runtime_error(
                "'initial_joint_positions' has " + std::to_string(initial.size()) + " values, the robot has " +
                std::to_string(joint_names_.size()) + " joints");
        }
        q_ = Eigen::Map<const Eigen::VectorXd>(initial.data(), static_cast<Eigen::Index>(initial.size()));
        if (!q_.allFinite()) {
            throw std::runtime_error("'initial_joint_positions' is not finite");
        }
        for (size_t i = 0; i < limits_.size(); ++i) {
            const auto k = static_cast<Eigen::Index>(i);
            if (q_[k] < limits_[i].min || q_[k] > limits_[i].max) {
                throw std::runtime_error("initial position of " + joint_names_[i] + " is outside its limits");
            }
        }
        // the plugin accepts the pose, and gives the tool position for the log below
        Eigen::Isometry3d tcp_pose;
        if (!kinematics_->calculate_link_transform(q_, tcp_link_, tcp_pose)) {
            throw std::runtime_error("the kinematics plugin rejects the initial pose / tcp link");
        }
        RCLCPP_INFO(
            get_logger(), "Start pose: tcp at (%.3f, %.3f, %.3f) m in the base frame",
            tcp_pose.translation().x(), tcp_pose.translation().y(), tcp_pose.translation().z());
    }

    void on_twist(const geometry_msgs::msg::Twist & msg)
    {
        Vector6 twist;
        twist << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;
        if (!twist.allFinite()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "Ignoring a non-finite twist");
            return;
        }
        // callbacks are serialized by the single threaded executor, no locking needed
        twist_ = twist;
        last_twist_ = Clock::now();
        has_twist_ = true;
    }

    void tick()
    {
        const auto now = Clock::now();
        // measured, but bounded: a stalled cycle must not turn into one huge jump
        const double dt =
            std::clamp(std::chrono::duration<double>(now - last_tick_).count(), 0.0, 3.0 * period_);
        last_tick_ = now;

        const bool twist_fresh =
            has_twist_ && std::chrono::duration<double>(now - last_twist_).count() <= twist_timeout_;

        dq_.setZero();
        if (twist_fresh && dt > 0.0 && twist_.squaredNorm() > 0.0) {
            step(dt);
        }
        publish(dt);
    }

    void step(double dt)
    {
        const Vector6 dx = twist_ * dt;
        if (!kinematics_->convert_cartesian_deltas_to_joint_deltas(q_, dx, tcp_link_, dq_)) {
            dq_.setZero();
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), kLogThrottleMs,
                "convert_cartesian_deltas_to_joint_deltas failed, holding position");
            return;
        }

        const auto limited = robotarm_kinematics::limit_joint_step(q_, dq_, limits_, dt);
        if (limited.reason == StepLimit::Invalid) {
            dq_.setZero();
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), kLogThrottleMs, "Joint step rejected (non-finite), holding position");
            return;
        }
        dq_ *= limited.scale;
        if (limited.scale < 1.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), kLogThrottleMs, "%s limit of %s: motion scaled to %.0f %%",
                limited.reason == StepLimit::Velocity ? "Velocity" : "Position",
                joint_names_[static_cast<size_t>(limited.joint)].c_str(), 100.0 * limited.scale);
        }

        q_ += dq_;
        // the limiter ends a step exactly at a limit, this only removes rounding
        for (size_t i = 0; i < limits_.size(); ++i) {
            const auto k = static_cast<Eigen::Index>(i);
            q_[k] = std::clamp(q_[k], limits_[i].min, limits_[i].max);
        }
    }

    void publish(double dt)
    {
        joint_state_.header.stamp = now();
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            const auto k = static_cast<Eigen::Index>(i);
            joint_state_.position[i] = q_[k];
            joint_state_.velocity[i] = dt > 0.0 ? dq_[k] / dt : 0.0;
        }
        joint_state_pub_->publish(joint_state_);
    }

    std::vector<std::string> joint_names_;
    std::vector<JointLimit> limits_;
    std::string tcp_link_;

    // the loader has to outlive the plugin instance: members are destroyed in reverse order
    std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
    pluginlib::UniquePtr<kinematics_interface::KinematicsInterface> kinematics_;

    double period_ = 0.01;
    double twist_timeout_ = 0.2;
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Vector6 twist_;
    bool has_twist_ = false;
    Clock::time_point last_twist_;
    Clock::time_point last_tick_;

    sensor_msgs::msg::JointState joint_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<CartesianJogNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("cartesian_jog"), "%s", e.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
