#include "robotarm_kinematics/KinematicsCore.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rclcpp/logging.hpp"

namespace
{
// Built on demand so there is no static-init ordering dependency on rclcpp.
rclcpp::Logger logger()
{
	return rclcpp::get_logger("robotarm_kinematics");
}
}

PLUGINLIB_EXPORT_CLASS(
  	robotarm_kinematics::KinematicsCore, 
  	kinematics_interface::KinematicsInterface
)

bool robotarm_kinematics::KinematicsCore::initialize(
  	const std::string &robot_description,
  	std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  	const std::string &param_namespace)
{
    joints_.clear();

    urdf::Model model;
    if (!model.initString(robot_description)) {
        RCLCPP_ERROR(logger(), "Failed to parse robot_description");
        return false;
    }

    // Walk root -> tip so joints_ ends up in kinematic order, not map/alphabetical order.
    urdf::LinkConstSharedPtr link_urdf = model.getRoot();
	if (!link_urdf) {
		RCLCPP_ERROR(logger(), "URDF has no root link");
		return false;
	}
	std::vector<urdf::JointConstSharedPtr> joints_urdf;

	while (!link_urdf->child_joints.empty()) {
		if (link_urdf->child_joints.size() != 1) {
			RCLCPP_ERROR(logger(), "Link %s has multiple children links", link_urdf->name.c_str());
			return false;
		}
		urdf::JointConstSharedPtr joint = link_urdf->child_joints[0];
        joints_urdf.push_back(joint);
        link_urdf = model.getLink(joint->child_link_name);
		if (!link_urdf) {
			RCLCPP_ERROR(logger(), "Joint %s references unknown child link %s",
				joint->name.c_str(), joint->child_link_name.c_str());
			return false;
		}
	}

	// Each DH row needs a joint and its successor, so size()-1 must not underflow.
	if (joints_urdf.size() < 2) {
		RCLCPP_ERROR(logger(), "URDF chain has %zu joint(s), need at least 2 to derive DH parameters",
			joints_urdf.size());
		return false;
	}

    for (size_t i = 0; i < joints_urdf.size()-1; ++i) {
		urdf::JointConstSharedPtr joint_urdf = joints_urdf[i];
		urdf::JointConstSharedPtr next_joint_urdf = joints_urdf[i+1];
  
		Joint joint;
		if (joint_urdf->type == urdf::Joint::REVOLUTE) {
			joint.is_fixed_ = false;
			if (!joint_urdf->limits) {
				RCLCPP_ERROR(logger(), "Joint %s has no limits", joint_urdf->name.c_str());
				return false;
			}
			const auto & l = *joint_urdf->limits;

			if (!(l.lower < l.upper)) {
				RCLCPP_ERROR(logger(), "Joint %s has invalid position limits", joint_urdf->name.c_str());
				return false;
			}
			if (l.velocity <= 0.0 || l.effort <= 0.0) {
				RCLCPP_ERROR(logger(), "Joint %s has invalid velocity/effort limits", joint_urdf->name.c_str());
				return false;
			}

			joint.limits_.min = l.lower;
			joint.limits_.max = l.upper;
			joint.limits_.velocity = l.velocity;
			joint.limits_.effort = l.effort;
		}
		else if (joint_urdf->type == urdf::Joint::FIXED) {
			joint.is_fixed_ = true;
		}
		else {
			RCLCPP_ERROR(logger(), "Joint %s has unexpected type", joint_urdf->name.c_str());
			return false;
		}
		
		joint.joint_name_ = joint_urdf->name;
		joint.parent_link_name_ = joint_urdf->parent_link_name;
		joint.child_link_name_ = joint_urdf->child_link_name;

		// extract dh params
		const double angular_eps = 1e-6;  // dimensionless, applied to sin(pitch)
		const double linear_eps = 1e-6;   // metres, applied to the residual translation
		double pitch, roll, yaw;
		double x, y, z;
		next_joint_urdf->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
		x = next_joint_urdf->parent_to_joint_origin_transform.position.x;
		y = next_joint_urdf->parent_to_joint_origin_transform.position.y;
		z = next_joint_urdf->parent_to_joint_origin_transform.position.z;

		Eigen::Matrix3d Rot = (
			Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();

		Eigen::Vector3d trans(x, y, z);

		// check for invalid rotation around y
		if (std::abs(pitch) > angular_eps) {
			RCLCPP_ERROR(logger(), "Joint %s not dh conform: invalid rotation",
				next_joint_urdf->name.c_str());
			return false;
		}

		// check for invalid translation in y
		Eigen::Vector3d trans_before_theta = Eigen::AngleAxisd(-yaw,  Eigen::Vector3d::UnitZ()).toRotationMatrix()
											* trans;
		if (std::abs(trans_before_theta[1]) > linear_eps) {
			RCLCPP_ERROR(logger(), "Joint %s not dh conform: invalid translation",
				next_joint_urdf->name.c_str());
			return false;
		}

		// already given in parent cs -> i-1
		joint.dhparams_.theta_0 = yaw;
		joint.dhparams_.d = z;

		// project trans onto child cs -> i
		Eigen::Vector3d x_i_in_im1 = Rot*Eigen::Vector3d::UnitX();
		joint.dhparams_.a = trans.dot(x_i_in_im1);

		// angle between z_i-1 and z_i projected on 2d plane
		Eigen::Vector3d z_im1_in_i = Rot.transpose()*Eigen::Vector3d::UnitZ();
		joint.dhparams_.alpha = atan2(z_im1_in_i[1], z_im1_in_i[2]);

		// get intertia
        urdf::LinkConstSharedPtr child_link_urdf = model.getLink(joint_urdf->child_link_name);
		if (!child_link_urdf) {
			RCLCPP_ERROR(logger(), "Joint %s references unknown child link %s",
				joint_urdf->name.c_str(), joint_urdf->child_link_name.c_str());
			return false;
		}
		if (child_link_urdf->inertial) {
			const auto & in = *child_link_urdf->inertial;

			if (in.mass <= 0.0) {
				RCLCPP_ERROR(logger(), "Link %s has invalid mass", child_link_urdf->name.c_str());
				return false;
			}

			InertialParams p;
			p.mass = in.mass;
			p.com = Eigen::Vector3d(in.origin.position.x, in.origin.position.y, in.origin.position.z);

			double com_roll, com_pitch, com_yaw;
			in.origin.rotation.getRPY(com_roll, com_pitch, com_yaw);
			p.com_rotation = (Eigen::AngleAxisd(com_yaw, Eigen::Vector3d::UnitZ()) *
								Eigen::AngleAxisd(com_pitch, Eigen::Vector3d::UnitY()) *
								Eigen::AngleAxisd(com_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

			p.inertia << in.ixx, in.ixy, in.ixz,
						in.ixy, in.iyy, in.iyz,
						in.ixz, in.iyz, in.izz;
			p.valid = true;
			joint.child_link_inertial_ = p;
		}

		joints_.push_back(joint);
    }

    print_joints();
    return true;
}

void robotarm_kinematics::KinematicsCore::print_joints() const
{
    constexpr double rad2deg = 180.0 / M_PI;

    size_t w_joint = std::string("joint").size();
    size_t w_parent = std::string("parent link").size();
    size_t w_child = std::string("child link").size();
    for (const auto & j : joints_) {
        w_joint = std::max(w_joint, j.joint_name_.size());
        w_parent = std::max(w_parent, j.parent_link_name_.size());
        w_child = std::max(w_child, j.child_link_name_.size());
    }

    // one value column: fixed width, so rad/deg columns line up
    const int w_val = 10;
    auto value = [&](std::ostringstream & os, double v) {
        os << std::setw(w_val) << v << "  ";
    };

    std::ostringstream os;
    os << std::fixed << std::setprecision(5);
    os << "\nKinematic chain: " << joints_.size() << " joint(s)"
       << "  (DH: a, d in m | alpha, theta_0 in rad and deg)\n";

    std::ostringstream header;
    header << std::left
           << std::setw(3) << "#" << "  "
           << std::setw(static_cast<int>(w_joint)) << "joint" << "  "
           << std::setw(static_cast<int>(w_parent)) << "parent link" << "  "
           << std::setw(static_cast<int>(w_child)) << "child link" << "  "
           << std::right
           << std::setw(w_val) << "a" << "  "
           << std::setw(w_val) << "alpha" << "  "
           << std::setw(w_val) << "alpha[deg]" << "  "
           << std::setw(w_val) << "d" << "  "
           << std::setw(w_val) << "theta_0" << "  "
           << std::setw(w_val) << "theta0[deg]";
    os << header.str() << "\n" << std::string(header.str().size(), '-') << "\n";

    for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        os << std::left
           << std::setw(3) << i << "  "
           << std::setw(static_cast<int>(w_joint)) << j.joint_name_ << "  "
           << std::setw(static_cast<int>(w_parent)) << j.parent_link_name_ << "  "
           << std::setw(static_cast<int>(w_child)) << j.child_link_name_ << "  "
           << std::right;
        value(os, j.dhparams_.a);
        value(os, j.dhparams_.alpha);
        value(os, j.dhparams_.alpha * rad2deg);
        value(os, j.dhparams_.d);
        value(os, j.dhparams_.theta_0);
        value(os, j.dhparams_.theta_0 * rad2deg);
        os << "\n";
    }

    RCLCPP_INFO(logger(), "%s", os.str().c_str());
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
