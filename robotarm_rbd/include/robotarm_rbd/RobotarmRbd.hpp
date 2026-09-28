#ifndef ROBOTARM_RBD_ROBOTARMRBD_HPP
#define ROBOTARM_RBD_ROBOTARMRBD_HPP

// ROS free rigid body model of an robotarm
//
// Errors: bool functions return false and leaves a message in last_error() without allocating heap
//
// Not thread safe: the scratch buffers and the error buffer are written on every call

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <urdf_model/joint.h>

namespace robotarm_rbd
{
    constexpr double angular_eps = 1e-6;  // dimensionless
	constexpr double linear_eps = 1e-6;   // metres, applied to the residual translation

class RobotarmData
{
public:
    std::vector<Eigen::Isometry3d> T;
    std::vector<Eigen::Vector3d> w;
    std::vector<Eigen::Vector3d> dot_w;
    std::vector<Eigen::Vector3d> a_org;
    std::vector<Eigen::Vector3d> a_com;
};

class RobotarmRbd
{
public:
    struct Limits {
        double effort = 0;
        double velocity = 0;
        double min = 0;
        double max = 0;
    };

    // valid is false if the link has no <inertial>, the other members then keep their defaults.
    struct InertialParams {
        double mass = 0;
        Eigen::Vector3d com = Eigen::Vector3d::Zero();                      // CoM position in joint frame
        Eigen::Matrix3d com_inertia_in_joint = Eigen::Matrix3d::Zero();     // about CoM, in joint axes
        bool valid = false;
    };

    struct Config {
        // damping factor of the damped least squares in calculate_jacobian_inverse(), >= 0
        double lambda = 0.01;
    };

protected:
    class Joint {
    public:
        std::string joint_name_;
        std::string parent_link_name_;
        std::string child_link_name_;
        Limits limits_;
        InertialParams inertia_;  // of child_link, the body this joint moves
        // joint origin = link cs of child_link
        Eigen::Isometry3d joint_origin_in_parent_ = Eigen::Isometry3d::Identity();
        Eigen::Vector3d joint_axis_in_child_ = Eigen::Vector3d::Zero();

    public:
        Eigen::Isometry3d transform(double theta) const {
            return joint_origin_in_parent_ * Eigen::AngleAxisd(theta, joint_axis_in_child_);
        }
    };

    class TCP {
    public:
        std::string tcp_name_;
        std::string parent_link_name_;
        Eigen::Isometry3d tcp_origin_in_parent_ = Eigen::Isometry3d::Identity();
    };

public:
    RobotarmRbd() = default;
    virtual ~RobotarmRbd() = default;

    bool initialize(const std::string &robot_description, const Config &config);

    bool convert_cartesian_deltas_to_joint_deltas(
        const Eigen::VectorXd &q,
        const Eigen::Matrix<double, 6, 1> &delta_x,
        const std::string &link_name,
        Eigen::VectorXd &delta_q);

    bool convert_joint_deltas_to_cartesian_deltas(
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &delta_q,
        const std::string &link_name,
        Eigen::Matrix<double, 6, 1> &delta_x);

    bool calculate_link_transform(
        const Eigen::VectorXd &q,
        const std::string &link_name,
        Eigen::Isometry3d &transform);

    bool calculate_jacobian(
        const Eigen::VectorXd &q,
        const std::string &link_name,
        Eigen::Matrix<double, 6,
        Eigen::Dynamic> &jacobian);
    
    bool calculate_jacobian_inverse(
        const Eigen::VectorXd &q,
        const std::string &link_name,
        Eigen::Matrix<double,
        Eigen::Dynamic, 6> &jacobian_inverse);

    bool recursive_newton_euler(
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq,
        const Eigen::VectorXd &ddq,
        Eigen::VectorXd &tau);

    bool initialised() const { return initialised_; }

    // Robot model, all return false before initialize(). Joints are in joint_pos order. Links are the
    // root plus the child of each joint (joint i connects link i and i+1), the tcp is not included.
    bool get_joint_names(std::vector<std::string>& names);
    bool get_joint_limits(std::vector<Limits>& limits);
    bool get_link_names(std::vector<std::string>& names);
    bool get_tcp_link_name(std::string& name);

    std::string chain_table_log() const;

    // Message of the last failure. Only meaningful right after a function returned false, it is
    // not cleared on success.
    const char * last_error() const { return error_; }

private:
    static Eigen::Isometry3d joint_origin_to_isometry(urdf::JointConstSharedPtr joint);
    bool check_q(const Eigen::VectorXd &q);
    // printf-style: writes the message into error_ (truncated to its size) and returns false
    bool fail(const char *fmt, ...) __attribute__((format(printf, 2, 3)));

protected:
    std::vector<Joint> joints_;
    TCP tcp_;

private:
    char error_[256] = "";
    // true once initialize() has run through; every kinematics function refuses to work before that
    bool initialised_ = false;

    // scratch buffers for the kinematics functions: results are built here and only copied
    // to the caller's output once complete, so a failure leaves the output untouched
    // (all or nothing). Sized in initialize(), so the rt path itself does not allocate.
    Eigen::Matrix<double, 6, Eigen::Dynamic> j_cj_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> j_cji_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> j_jd2cd_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> j_inv_cd2jd_;
    Eigen::MatrixXd M_;
    Eigen::LDLT<Eigen::MatrixXd> ldlt_;

    Config config_;
};

}

#endif
