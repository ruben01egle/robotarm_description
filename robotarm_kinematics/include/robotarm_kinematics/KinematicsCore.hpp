#ifndef ROBOTARM_KINEMATICS_KINEMATICSCORE_HPP
#define ROBOTARM_KINEMATICS_KINEMATICSCORE_HPP

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

namespace robotarm_kinematics
{

class KinematicsCore : public kinematics_interface::KinematicsInterface
{
protected:
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
    struct InertialParams {
        double mass = 0;
        Eigen::Vector3d com = Eigen::Vector3d::Zero();
        Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d com_rotation = Eigen::Matrix3d::Identity();
        bool valid = false;
    };


    class Joint {
    public:
        std::string joint_name_;
        // i-1 link
        std::string parent_link_name_;
        // i link
        std::string child_link_name_;
        Limits limits_;
        InertialParams child_link_inertial_;
        DHParams dhparams_;
        bool is_fixed_ = false;
    };

public:
    KinematicsCore() = default;
    virtual ~KinematicsCore() = default;

    virtual bool initialize(
        const std::string &robot_description,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
        const std::string &param_namespace) override;

    // Logs every parsed joint (name, link names, DH params) as a table.
    void print_joints() const;

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

protected:
    std::vector<Joint> joints_;

private:
};

}

#endif