#ifndef ROBOTARM_RBD_TESTABLE_RBD_HPP
#define ROBOTARM_RBD_TESTABLE_RBD_HPP

// Exposes the parsed chain of RobotarmRbd to the tests. ROS free, so robotarm_rbd_test can use it
// without linking rclcpp.

#include "robotarm_rbd/RobotarmRbd.hpp"

namespace test_utils
{

class TestableRbd : public robotarm_rbd::RobotarmRbd
{
public:
    using RobotarmRbd::joints_;
    using RobotarmRbd::tcp_;
};

}  // namespace test_utils

#endif  // ROBOTARM_RBD_TESTABLE_RBD_HPP
