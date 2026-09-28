// Tests for RobotarmRbd. Linked without rclcpp on purpose: if the library pulls in anything from ROS,
// this test stops linking. The full URDF rejection matrix runs through the Kinematics wrapper in
// kinematics_test.cpp, here only the error reporting and the lifecycle of the ROS free class.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "robotarm_rbd/RobotarmRbd.hpp"
#include "testable_rbd.hpp"

namespace
{

// two revolute joints and the tcp; axis2 is passed in so a test can break it
std::string two_joint_urdf(const std::string & axis2 = "0 1 0")
{
    return R"(<robot name="two_joint">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2">
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0.01" ixz="0.02" iyz="0.03"/>
      <origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
    </inertial>
  </link>
  <link name="tcp"/>
  <joint name="axis1" type="revolute">
    <parent link="base"/> <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/> <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/>
  </joint>
  <joint name="axis2" type="revolute">
    <parent link="link1"/> <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/> <axis xyz=")" + axis2 + R"("/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="link2"/> <child link="tcp"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>)";
}

}  // namespace

TEST(RobotarmRbd, AValidUrdfInitialises)
{
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {})) << rbd.last_error();
    EXPECT_TRUE(rbd.initialised());

    std::vector<std::string> names;
    std::string tcp;
    ASSERT_TRUE(rbd.get_joint_names(names));
    ASSERT_TRUE(rbd.get_tcp_link_name(tcp));
    EXPECT_EQ(names, (std::vector<std::string>{"axis1", "axis2"}));
    EXPECT_EQ(tcp, "tcp");
}

TEST(RobotarmRbd, InertiaIsParsedFromTheChildLinkIfPresent)
{
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {})) << rbd.last_error();
    ASSERT_EQ(rbd.joints_.size(), 2u);

    // link1 has no <inertial>: left as constructed
    const auto & none = rbd.joints_[0].intertia_;
    EXPECT_FALSE(none.valid);
    EXPECT_EQ(none.mass, 0.0);
    EXPECT_TRUE(none.com.isZero());
    EXPECT_TRUE(none.inertia.isZero());
    EXPECT_TRUE(none.com_rotation.isIdentity());

    const auto & in = rbd.joints_[1].intertia_;
    EXPECT_TRUE(in.valid);
    EXPECT_DOUBLE_EQ(in.mass, 2.5);
    EXPECT_TRUE(in.com.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
    Eigen::Matrix3d I;
    I << 0.1, 0.01, 0.02,
         0.01, 0.2, 0.03,
         0.02, 0.03, 0.3;
    EXPECT_TRUE(in.inertia.isApprox(I));
    EXPECT_TRUE(in.com_rotation.isApprox(
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix(), 1e-12));
}

TEST(RobotarmRbd, AMalformedUrdfFailsWithAMessage)
{
    robotarm_rbd::RobotarmRbd rbd;
    EXPECT_FALSE(rbd.initialize("<robot", {}));
    EXPECT_FALSE(rbd.initialised());
    EXPECT_STRNE(rbd.last_error(), "");
}

TEST(RobotarmRbd, AnInvalidLambdaFailsWithAMessage)
{
    for (const double lambda : {-0.1, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity()})
    {
        SCOPED_TRACE("lambda = " + std::to_string(lambda));
        robotarm_rbd::RobotarmRbd rbd;
        EXPECT_FALSE(rbd.initialize(two_joint_urdf(), {lambda}));
        EXPECT_FALSE(rbd.initialised());
        EXPECT_NE(std::string(rbd.last_error()).find("lambda"), std::string::npos) << rbd.last_error();
    }
}

TEST(RobotarmRbd, TheMessageNamesTheOffendingJoint)
{
    robotarm_rbd::RobotarmRbd rbd;
    EXPECT_FALSE(rbd.initialize(two_joint_urdf("0 0 0"), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("axis2"), std::string::npos) << rbd.last_error();
}

TEST(RobotarmRbd, AFailedReinitialiseLeavesTheObjectUnusable)
{
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {}));
    EXPECT_FALSE(rbd.initialize("<robot", {}));
    EXPECT_FALSE(rbd.initialised());
    std::vector<std::string> names;
    EXPECT_FALSE(rbd.get_joint_names(names));
}

TEST(RobotarmRbd, GettersFailBeforeInitialiseWithAMessage)
{
    robotarm_rbd::RobotarmRbd rbd;
    std::vector<std::string> names{"keep"};
    EXPECT_FALSE(rbd.get_joint_names(names));
    EXPECT_STREQ(rbd.last_error(), "Not initialised");
    EXPECT_EQ(names, std::vector<std::string>{"keep"});
}
