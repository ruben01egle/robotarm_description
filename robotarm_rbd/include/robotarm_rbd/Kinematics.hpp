#ifndef ROBOTARM_RBD_KINEMATICS_HPP
#define ROBOTARM_RBD_KINEMATICS_HPP

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"

#include "robotarm_rbd/RobotarmRbd.hpp"

namespace robotarm_rbd
{
	constexpr int log_throttle_ms = 1000; // period of error logs in functions called from the rt loop

class Kinematics : public kinematics_interface::KinematicsInterface
{
public:
    using Limits = RobotarmRbd::Limits;
    // Caller contract of the kinematics functions below:
    //  - inputs (joint_pos, delta_x, delta_theta) must be finite, and joint_pos / delta_theta must have
    //    one entry per joint, otherwise the call logs an error, returns false and leaves the output untouched.
    //  - dynamic-size outputs (jacobian 6xN, jacobian_inverse Nx6, delta_theta N) are resized by Eigen
    //    if they do not fit. That allocates, so rt callers should pass pre-sized objects to stay
    //    allocation free (the std::vector overloads of the base class allocate regardless).
    Kinematics() = default;
    virtual ~Kinematics() = default;

    virtual bool initialize(
        const std::string &robot_description,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
        const std::string &param_namespace) override;

    virtual bool convert_cartesian_deltas_to_joint_deltas(
        const Eigen::VectorXd &joint_pos,
        const Eigen::Matrix<double, 6, 1> &delta_x,
        const std::string &link_name,
        Eigen::VectorXd &delta_theta) override;

    virtual bool convert_joint_deltas_to_cartesian_deltas(
        const Eigen::VectorXd &joint_pos,
        const Eigen::VectorXd &delta_theta,
        const std::string &link_name,
        Eigen::Matrix<double, 6, 1> &delta_x) override;

    virtual bool calculate_link_transform(
        const Eigen::VectorXd &joint_pos,
        const std::string &link_name,
        Eigen::Isometry3d &transform) override;

    virtual bool calculate_jacobian(
        const Eigen::VectorXd &joint_pos,
        const std::string &link_name,
        Eigen::Matrix<double, 6,
        Eigen::Dynamic> &jacobian) override;
    
    virtual bool calculate_jacobian_inverse(
        const Eigen::VectorXd &joint_pos,
        const std::string &link_name,
        Eigen::Matrix<double,
        Eigen::Dynamic, 6> &jacobian_inverse) override;

    // Robot model, all return false before initialize(). Joints are in joint_pos order. Links are the
    // root plus the child of each joint (joint i connects link i and i+1), the tcp is not included.
    bool get_joint_names(std::vector<std::string>& names);
    bool get_joint_limits(std::vector<Limits>& limits);
    bool get_link_names(std::vector<std::string>& names);
    bool get_tcp_link_name(std::string& name);

    // Logs the parsed chain (joint and link names, root -> tcp) as a table.
    void print_joints() const;

private:
    RobotarmRbd rbd_;

    // steady clock for the throttled error logs in the rt path
    rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}

#endif