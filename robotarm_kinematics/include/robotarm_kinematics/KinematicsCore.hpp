#ifndef ROBOTARM_KINEMATICS_KINEMATICSCORE_HPP
#define ROBOTARM_KINEMATICS_KINEMATICSCORE_HPP

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"
#include "urdf/model.h"

namespace robotarm_kinematics
{

    constexpr double angular_eps = 1e-6;  // dimensionless
	constexpr double linear_eps = 1e-6;   // metres, applied to the residual translation
	constexpr int log_throttle_ms = 1000; // period of error logs in functions called from the rt loop

class KinematicsCore : public kinematics_interface::KinematicsInterface
{
public:
    struct DHParams {
        double a = 0;
        double alpha = 0;
        double d = 0;
        double theta_0 = 0;
    };

    struct Limits {
        double effort = 0;
        double velocity = 0;
        double min = 0;
        double max = 0;
    };

protected:
    class Joint {
    public:
        std::string joint_name_;
        // i-1 link
        std::string parent_link_name_;
        // i link
        std::string child_link_name_;
        Limits limits_;
        DHParams dhparams_;
        Eigen::Isometry3d child_urdf_frame_in_child_dh_frame_ = Eigen::Isometry3d::Identity();
    };

public:
    // Caller contract of the kinematics functions below:
    //  - inputs (joint_pos, delta_x, delta_theta) must be finite, and joint_pos / delta_theta must have
    //    one entry per joint, otherwise the call logs an error, returns false and leaves the output untouched.
    //  - dynamic-size outputs (jacobian 6xN, jacobian_inverse Nx6, delta_theta N) are resized by Eigen
    //    if they do not fit. That allocates, so rt callers should pass pre-sized objects to stay
    //    allocation free (the std::vector overloads of the base class allocate regardless).
    KinematicsCore() = default;
    virtual ~KinematicsCore() = default;

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

    Eigen::Isometry3d dh_params_to_isometry(DHParams dhparams, double theta=0);

    // Robot model, all return false before initialize(). Joints are in joint_pos order. Links are the
    // root plus the child of each joint (joint i connects link i and i+1), the tcp is not included.
    bool get_joint_names(std::vector<std::string>& names);
    bool get_joint_limits(std::vector<Limits>& limits);
    bool get_link_names(std::vector<std::string>& names);
    bool get_tcp_link_name(std::string& name);

    // Logs every parsed joint (name, link names, DH params) as a table.
    void print_joints() const;

protected:
    std::vector<Joint> joints_;
    std::string tcp_link_name_;

private:
    // Common input validation of the kinematics functions: initialised, joint_pos has one entry
    // per joint and is finite. Logs (throttled) and returns false if any of that is violated.
    bool check_joint_pos(const Eigen::VectorXd &joint_pos);

    // true once initialize() has run through; every kinematics function refuses to work before that
    bool initialised_ = false;

    // steady clock for the throttled error logs in the rt path
    rclcpp::Clock clock_{RCL_STEADY_TIME};

    // scratch buffers for the kinematics functions: results are built here and only copied
    // to the caller's output once complete, so a failure leaves the output untouched
    // (all or nothing). Sized in initialize(), so the rt path itself does not allocate.
    Eigen::Matrix<double, 6, Eigen::Dynamic> j_cj_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> j_cji_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> j_jd2cd_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> j_inv_cd2jd_;
    Eigen::MatrixXd M_;
    Eigen::LDLT<Eigen::MatrixXd> ldlt_;

    // damping factor of the damped least squares in calculate_jacobian_inverse(),
    // read from the "<param_namespace>.lambda" parameter in initialize()
    double lambda_ = 0.01;
};

}

#endif