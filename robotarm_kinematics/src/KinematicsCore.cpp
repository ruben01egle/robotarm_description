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
    initialised_ = false;
    joints_.clear();

    // Declared only if the host node has not already declared it (or auto-declared it from overrides).
    const std::string lambda_param = (param_namespace.empty() ? "" : param_namespace + ".") + "lambda";
    try {
        if (!parameters_interface->has_parameter(lambda_param)) {
            parameters_interface->declare_parameter(lambda_param, rclcpp::ParameterValue(0.01));
        }
        lambda_ = parameters_interface->get_parameter(lambda_param).as_double();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Failed to read parameter '%s': %s", lambda_param.c_str(), e.what());
        return false;
    }
    if (!std::isfinite(lambda_) || lambda_ < 0.0) {
        RCLCPP_ERROR(logger(), "Parameter '%s' must be finite and >= 0", lambda_param.c_str());
        return false;
    }

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

	if (joints_urdf.size() != 7) {
		RCLCPP_ERROR(logger(), "URDF chain has %zu joint(s), expected 7 (6 dof + tcp)",
			joints_urdf.size());
		return false;
	}

	// DH convention: base frame == world frame, so the first joint must not offset/rotate it.
	const urdf::Pose & first_origin = joints_urdf[0]->parent_to_joint_origin_transform;
	if (std::abs(first_origin.position.x) > linear_eps ||
		std::abs(first_origin.position.y) > linear_eps ||
		std::abs(first_origin.position.z) > linear_eps) {
		RCLCPP_ERROR(logger(), "Joint %s not dh conform: first joint origin must have no translation",
			joints_urdf[0]->name.c_str());
		return false;
	}
	double first_roll, first_pitch, first_yaw;
	first_origin.rotation.getRPY(first_roll, first_pitch, first_yaw);
	if (std::abs(first_roll) > angular_eps ||
		std::abs(first_pitch) > angular_eps ||
		std::abs(first_yaw) > angular_eps) {
		RCLCPP_ERROR(logger(), "Joint %s not dh conform: first joint origin must have no rotation",
			joints_urdf[0]->name.c_str());
		return false;
	}

	urdf::JointConstSharedPtr tcp_joint = joints_urdf.back();
	if (tcp_joint->type != urdf::Joint::FIXED) {
		RCLCPP_ERROR(logger(), "TCP-Joint %s has unexpected type", tcp_joint->name.c_str());
		return false;
	}
	tcp_link_name_ = tcp_joint->child_link_name;

    for (size_t i = 0; i < joints_urdf.size()-1; ++i) {
		urdf::JointConstSharedPtr joint_urdf = joints_urdf[i];
		urdf::JointConstSharedPtr next_joint_urdf = joints_urdf[i+1];
  
		Joint joint;
		if (joint_urdf->type == urdf::Joint::REVOLUTE) {
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

			// DH joint rotates about +z of its own frame
			const urdf::Vector3 & axis = joint_urdf->axis;
			if (std::abs(axis.x) > angular_eps ||
				std::abs(axis.y) > angular_eps ||
				std::abs(axis.z - 1.0) > angular_eps) {
				RCLCPP_ERROR(logger(), "Joint %s not dh conform: axis must be (0, 0, 1)",
					joint_urdf->name.c_str());
				return false;
			}

			joint.limits_.min = l.lower;
			joint.limits_.max = l.upper;
			joint.limits_.velocity = l.velocity;
			joint.limits_.effort = l.effort;
		}
		else {
			RCLCPP_ERROR(logger(), "Joint %s has unexpected type", joint_urdf->name.c_str());
			return false;
		}
		
		joint.joint_name_ = joint_urdf->name;
		joint.parent_link_name_ = joint_urdf->parent_link_name;
		joint.child_link_name_ = joint_urdf->child_link_name;

		// extract dh params
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

		// store inv(T(theta=0)) to convert from dh cs-placement convention to urdf
		joint.child_urdf_frame_in_child_dh_frame_ = dh_params_to_isometry(joint.dhparams_).inverse();

		// check kinematic chain
        urdf::LinkConstSharedPtr child_link_urdf = model.getLink(joint_urdf->child_link_name);
		if (!child_link_urdf) {
			RCLCPP_ERROR(logger(), "Joint %s references unknown child link %s",
				joint_urdf->name.c_str(), joint_urdf->child_link_name.c_str());
			return false;
		}

		joints_.push_back(joint);
    }

	// init heap member
	j_cj_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_cji_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_jd2cd_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_inv_cd2jd_ =  Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(joints_.size(), 6);
	M_ = Eigen::MatrixXd::Zero(joints_.size(), joints_.size());
	ldlt_ = Eigen::LDLT<Eigen::MatrixXd>(joints_.size());

    print_joints();
    initialised_ = true;
    return true;
}

bool robotarm_kinematics::KinematicsCore::convert_cartesian_deltas_to_joint_deltas(
	const Eigen::VectorXd &joint_pos,
  	const Eigen::Matrix<double, 6, 1> &delta_x,
  	const std::string &link_name,
  	Eigen::VectorXd &delta_theta)
{
	if (!check_joint_pos(joint_pos)) return false;

	if (!delta_x.allFinite()) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "delta_x contains NaN or inf");
		return false;
	}

	// delta_theta = J⁺ delta_x
	if (!calculate_jacobian_inverse(joint_pos, link_name, j_inv_cd2jd_)) return false;
	delta_theta.noalias() = j_inv_cd2jd_ * delta_x;
    return true;
}

bool robotarm_kinematics::KinematicsCore::convert_joint_deltas_to_cartesian_deltas(
  	const Eigen::VectorXd &joint_pos,
  	const Eigen::VectorXd &delta_theta,
  	const std::string &link_name,
  	Eigen::Matrix<double, 6, 1> &delta_x)
{
	// checks initialised_ first, which the size check of delta_theta below relies on
	if (!check_joint_pos(joint_pos)) return false;

	if (delta_theta.size() != static_cast<Eigen::Index>(joints_.size())) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Unexpected delta_theta dimension");
		return false;
	}
	if (!delta_theta.allFinite()) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "delta_theta contains NaN or inf");
		return false;
	}

	// delta_x = J delta_theta
	if (!calculate_jacobian(joint_pos, link_name, j_jd2cd_)) return false;
	delta_x.noalias() = j_jd2cd_ * delta_theta;
    return true;
}

bool robotarm_kinematics::KinematicsCore::check_joint_pos(const Eigen::VectorXd &joint_pos)
{
	if (!initialised_) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Kinematics not initialised");
		return false;
	}

	if (joint_pos.size() != static_cast<Eigen::Index>(joints_.size())) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Unexpected joint_pos dimension");
		return false;
	}

	if (!joint_pos.allFinite()) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "joint_pos contains NaN or inf");
		return false;
	}

	return true;
}

bool robotarm_kinematics::KinematicsCore::calculate_link_transform(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Isometry3d &transform)
{
    // The DH chain starts in the root link's URDF frame only because initialize()
    // enforces an identity origin on the first joint (DH frame 0 == root link frame).

	if (!check_joint_pos(joint_pos)) return false;

	if (link_name == joints_.front().parent_link_name_) {
		transform = Eigen::Isometry3d::Identity();
		return true;
	}

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	for (size_t i = 0; i < joints_.size(); ++i) {
		T = T * dh_params_to_isometry(joints_[i].dhparams_, joint_pos[i]);
		if (joints_[i].child_link_name_ == link_name) {
			transform = T * joints_[i].child_urdf_frame_in_child_dh_frame_;
			return true;
		}
	}

	if (link_name == tcp_link_name_) {  // TCP is not in joints_, so it comes after the loop
		transform = T;
		return true;
	}

	RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms,
		"Link %s not found in kinematic chain", link_name.c_str());
	return false;

}

bool robotarm_kinematics::KinematicsCore::calculate_jacobian(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian)
{
	if (!check_joint_pos(joint_pos)) return false;

	if (link_name == joints_.front().parent_link_name_) {
		jacobian.setZero(6, joints_.size());
		return true;
	}

	j_cj_.setZero();
	Eigen::Isometry3d T_end;
	if (!calculate_link_transform(joint_pos, link_name, T_end)) return false;
	Eigen::Vector3d p_end = T_end.translation();

	for (size_t i = 0; i < joints_.size(); ++i) {

		Eigen::Isometry3d T_i;
		if (!calculate_link_transform(joint_pos, joints_[i].child_link_name_, T_i)) return false;
		Eigen::Vector3d z_axis = T_i.linear().col(2);
		Eigen::Vector3d p_i = T_i.translation();

		j_cj_.block<3,1>(0, i) = z_axis.cross(p_end - p_i);
    	j_cj_.block<3,1>(3, i) = z_axis;

		if (joints_[i].child_link_name_ == link_name) {
			break;
		}
	}

	jacobian = j_cj_;
    return true;
}

bool robotarm_kinematics::KinematicsCore::calculate_jacobian_inverse(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobian_inverse)
{
	// joint_pos is validated by calculate_jacobian(), which is the first thing called here
	if (!calculate_jacobian(joint_pos, link_name, j_cji_)) return false;

	// calculate J⁻¹ as pseudo-inverse with damped least squares: f(q̇) = ‖J q̇ − ẋ‖² + λ²‖q̇‖²
	// J⁺ = (JᵀJ + λ²I)⁻¹ Jᵀ -> (JᵀJ + λ²I)J⁺ = Jᵀ as MX = Jᵀ with:
	// M = (JᵀJ + λ²I)
	// X = J⁺
	M_.noalias() = j_cji_.transpose() * j_cji_;
	M_.diagonal().array() += lambda_*lambda_;

	// factor M once, and check that it worked
	ldlt_.compute(M_);
	if (ldlt_.info() != Eigen::Success) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Factorization failed");
		return false;
	}
	jacobian_inverse = ldlt_.solve(j_cji_.transpose());

    return true;
}

Eigen::Isometry3d robotarm_kinematics::KinematicsCore::dh_params_to_isometry(DHParams dhparams, double theta)
{
	Eigen::Isometry3d child_frame_in_parent_frame = Eigen::Isometry3d::Identity()
												  * Eigen::AngleAxisd(dhparams.theta_0+theta, Eigen::Vector3d::UnitZ())
												  * Eigen::Translation3d(0, 0, dhparams.d)
												  * Eigen::Translation3d(dhparams.a, 0, 0)
												  * Eigen::AngleAxisd(dhparams.alpha, Eigen::Vector3d::UnitX());
    return child_frame_in_parent_frame;
}

bool robotarm_kinematics::KinematicsCore::get_joint_names(std::vector<std::string>& names)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	names.clear();
	names.reserve(joints_.size());
	for (const auto & j : joints_) {
		names.push_back(j.joint_name_);
	}
    return true;
}

bool robotarm_kinematics::KinematicsCore::get_joint_limits(std::vector<Limits>& limits)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	limits.clear();
	limits.reserve(joints_.size());
	for (const auto & j : joints_) {
		limits.push_back(j.limits_);
	}
    return true;
}

bool robotarm_kinematics::KinematicsCore::get_link_names(std::vector<std::string> &names)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	names.clear();
	names.reserve(joints_.size()+1);
	names.push_back(joints_.front().parent_link_name_);
	for (const auto & j : joints_) {
		names.push_back(j.child_link_name_);
	}
    return true;
}

bool robotarm_kinematics::KinematicsCore::get_tcp_link_name(std::string& name)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	name = tcp_link_name_;
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

    // one value column: fixed width, so columns line up
    const int w_val = 11;
    auto value = [&](std::ostringstream & os, double v) {
        os << std::setw(w_val) << v << "  ";
    };

    std::ostringstream os;
    os << std::fixed << std::setprecision(5);
    os << "\nKinematic chain: " << joints_.size() << " joint(s)"
       << "  (DH: a, d in m | alpha, theta_0 in deg)\n";

    std::ostringstream header;
    header << std::left
           << std::setw(3) << "#" << "  "
           << std::setw(static_cast<int>(w_joint)) << "joint" << "  "
           << std::setw(static_cast<int>(w_parent)) << "parent link" << "  "
           << std::setw(static_cast<int>(w_child)) << "child link" << "  "
           << std::right
           << std::setw(w_val) << "a" << "  "
           << std::setw(w_val) << "alpha[deg]" << "  "
           << std::setw(w_val) << "d" << "  "
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
        value(os, j.dhparams_.alpha * rad2deg);
        value(os, j.dhparams_.d);
        value(os, j.dhparams_.theta_0 * rad2deg);
        os << "\n";
    }

    RCLCPP_INFO(logger(), "%s", os.str().c_str());
}


