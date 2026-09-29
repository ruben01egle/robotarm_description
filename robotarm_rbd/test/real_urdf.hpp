#ifndef ROBOTARM_RBD_REAL_URDF_HPP
#define ROBOTARM_RBD_REAL_URDF_HPP

// The URDF of the real robot, processed by xacro. Needs only ament_index_cpp (no rclcpp), so the ROS
// free tests can use it too.

#include <cstdio>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace test_utils
{

// URDF of the real robot. Throws if robotarm_description or xacro is not available: both are
// exec_depends of this package, so that is an error of the environment, not something to skip.
inline std::string real_urdf()
{
    static const std::string cached = [] {
        const std::string share = ament_index_cpp::get_package_share_directory("robotarm_description");
        const std::string cmd = "xacro '" + share + "/urdf/robotarm.urdf.xacro' 2>/dev/null";
        FILE * pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("cannot run xacro");
        }
        std::string out;
        char buf[4096];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), pipe)) > 0) {
            out.append(buf, n);
        }
        if (pclose(pipe) != 0 || out.empty()) {
            throw std::runtime_error("xacro failed on " + share + "/urdf/robotarm.urdf.xacro");
        }
        return out;
    }();
    return cached;
}

}  // namespace test_utils

#endif  // ROBOTARM_RBD_REAL_URDF_HPP
