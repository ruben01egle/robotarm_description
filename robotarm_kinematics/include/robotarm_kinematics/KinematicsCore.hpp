#ifndef ROBOTARM_KINEMATICS_KINEMATICSCORE_HPP
#define ROBOTARM_KINEMATICS_KINEMATICSCORE_HPP

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robotarm_kinematics
{

class KinematicsCore : public kinematics_interface::KinematicsInterface
{
public:
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

private:
};

}

#endif