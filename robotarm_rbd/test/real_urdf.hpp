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

// The real URDF with a gripper between flange and tcp, all joints fixed (Pinocchio merges them like
// we fold them): a base with a rotated inertial and off-diagonal tensor on a rotated attach joint,
// two jaws on rotated joints (a branch), and the tcp offset and rotated on the base.
inline std::string real_urdf_with_test_tool()
{
    const std::string tool = R"(
  <link name="test_tool_base">
    <inertial><origin xyz="0.01 -0.02 0.03" rpy="0.3 -0.2 0.5"/><mass value="0.6"/>
      <inertia ixx="0.002" ixy="0.0003" ixz="-0.0001" iyy="0.003" iyz="0.0002" izz="0.0025"/></inertial>
  </link>
  <link name="test_tool_jaw1">
    <inertial><origin xyz="0 0.01 0.02" rpy="0 0 0"/><mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="2e-5" iyz="0" izz="1.5e-5"/></inertial>
  </link>
  <link name="test_tool_jaw2">
    <inertial><origin xyz="0 -0.01 0.02" rpy="0.1 0 0"/><mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="2e-5" iyz="0" izz="1.5e-5"/></inertial>
  </link>
  <joint name="test_tool_attach_joint" type="fixed">
    <origin xyz="0 0 0.005" rpy="0 0 0.7"/><parent link="flange"/><child link="test_tool_base"/>
  </joint>
  <joint name="test_tool_jaw1_joint" type="fixed">
    <origin xyz="0 0.03 0.06" rpy="0 0.2 0"/><parent link="test_tool_base"/><child link="test_tool_jaw1"/>
  </joint>
  <joint name="test_tool_jaw2_joint" type="fixed">
    <origin xyz="0 -0.03 0.06" rpy="0 -0.2 0"/><parent link="test_tool_base"/><child link="test_tool_jaw2"/>
  </joint>
  <joint name="test_tool_tcp_joint" type="fixed">
    <origin xyz="0.01 0 0.11" rpy="0.2 0 -0.4"/><parent link="test_tool_base"/><child link="tcp"/>
  </joint>
)";
    // drop the tcp joint of tools/none.xacro (flange -> tcp), the tool brings its own
    std::string urdf = real_urdf();
    const size_t child = urdf.find("<child link=\"tcp\"/>");
    const size_t begin = urdf.rfind("<joint ", child);
    const size_t end = urdf.find("</joint>", child);
    if (child == std::string::npos || begin == std::string::npos || end == std::string::npos) {
        throw std::runtime_error("real URDF: no tcp joint to replace");
    }
    urdf.erase(begin, end + std::string("</joint>").size() - begin);
    urdf.insert(urdf.rfind("</robot>"), tool);
    return urdf;
}

}  // namespace test_utils

#endif  // ROBOTARM_RBD_REAL_URDF_HPP
