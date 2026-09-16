#include "robotarm_kinematics/KinematicsCore.hpp"

PLUGINLIB_EXPORT_CLASS(
  robotarm_kinematics::KinematicsCore, 
  kinematics_interface::KinematicsInterface
)

bool robotarm_kinematics::KinematicsCore::initialize(
  const std::string &robot_description,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string &param_namespace)
{
    return false;
}

bool robotarm_kinematics::KinematicsCore::convert_cartesian_deltas_to_joint_deltas(
  const Eigen::VectorXd &joint_pos,
  const Eigen::Matrix<double, 6, 1> &delta_x,
  const std::string &link_name,
  Eigen::VectorXd &delta_theta)
{
    return false;
}

bool robotarm_kinematics::KinematicsCore::convert_joint_deltas_to_cartesian_deltas(
  const Eigen::VectorXd &joint_pos,
  const Eigen::VectorXd &delta_theta,
  const std::string &link_name,
  Eigen::Matrix<double, 6, 1> &delta_x)
{
    return false;
}

bool robotarm_kinematics::KinematicsCore::calculate_link_transform(
  const Eigen::VectorXd &joint_pos,
  const std::string &link_name,
  Eigen::Isometry3d &transform)
{
    return false;
}

bool robotarm_kinematics::KinematicsCore::calculate_jacobian(
  const Eigen::VectorXd &joint_pos,
  const std::string &link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian)
{
    return false;
}

bool robotarm_kinematics::KinematicsCore::calculate_jacobian_inverse(
  const Eigen::VectorXd &joint_pos,
  const std::string &link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobian_inverse)
{
    return false;
}
