#include "robotarm_kinematics/Kinematics.hpp"
/*
    // Structure the closed-form IK relies on. Row k is joint k (joints_[k-1]); only the
	// zeros and right angles are checked, the link lengths stay free.
	auto dh = [&](size_t k) -> const DHParams & { return joints_[k - 1].dhparams_; };
	auto is_zero = [](double v, double eps) { return std::abs(v) <= eps; };
	auto is_right_angle = [&](double alpha) {
		return std::abs(std::abs(alpha) - M_PI / 2.0) <= angular_eps;
	};
	auto require = [&](bool ok, const char * what) {
		if (!ok) {
			RCLCPP_ERROR(logger(), "Kinematic structure not supported by the IK: %s", what);
		}
		return ok;
	};

	if (!require(is_right_angle(dh(1).alpha), "axis 1 must be perpendicular to axis 2") ||
		!require(is_zero(dh(2).alpha, angular_eps), "axes 2 and 3 must be parallel") ||
		!require(is_right_angle(dh(3).alpha), "axis 3 must be perpendicular to axis 4") ||
		!require(is_zero(dh(4).a, linear_eps) && is_right_angle(dh(4).alpha),
			"axes 4 and 5 must intersect at a right angle") ||
		!require(is_zero(dh(5).a, linear_eps) && is_right_angle(dh(5).alpha),
			"axes 5 and 6 must intersect at a right angle") ||
		!require(is_zero(dh(5).d, linear_eps), "axes 4, 5 and 6 must meet in one point (spherical wrist)")) {
		return false;
	}
*/