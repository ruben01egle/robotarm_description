// The real-time path of KinematicsCore must not allocate when the caller passes pre-sized outputs.
//
// Eigen can turn heap allocations into failed assertions (EIGEN_RUNTIME_NO_MALLOC, see
// eigen_malloc_guard.hpp). The guard has to be compiled into the code under test, so this target
// compiles src/KinematicsCore.cpp itself instead of linking the library.
//
// Every "must not allocate" check has a negative control that must allocate. Without those a guard
// that is silently switched off (e.g. by NDEBUG) would let all checks pass.

#include <gtest/gtest.h>

#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "test_utils.hpp"

namespace
{

using test_utils::Jacobian;
using test_utils::JacobianInverse;
using test_utils::Vector6;

// Runs f with Eigen heap allocations forbidden. Returns true if f tried to allocate. Any other
// failure (a different Eigen assertion, an exception) is not swallowed.
template<class F>
bool allocates(F && f)
{
    struct Guard
    {
        Guard() {Eigen::internal::set_is_malloc_allowed(false);}
        ~Guard() {Eigen::internal::set_is_malloc_allowed(true);}
    } guard;
    try {
        f();
    } catch (const std::runtime_error & e) {
        if (std::string(e.what()).find("is_malloc_allowed") != std::string::npos) {
            return true;
        }
        throw;
    }
    return false;
}

class MallocTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        urdf_ = test_utils::make_spec(test_utils::robot_dh()).str();
        ref_ = std::make_unique<test_utils::ReferenceFk>(urdf_);
        ASSERT_TRUE(test_utils::initialize(core_, urdf_));
        rng_.seed(4711);
    }

    test_utils::TestableCore core_;
    std::string urdf_;
    std::unique_ptr<test_utils::ReferenceFk> ref_;
    std::mt19937 rng_;
};

TEST_F(MallocTest, PresizedCallsDoNotAllocateForAnyLink)
{
    Eigen::Isometry3d T;
    Jacobian J = Jacobian::Zero(6, 6);
    JacobianInverse Ji = JacobianInverse::Zero(6, 6);
    Vector6 dx = Vector6::Zero();
    Eigen::VectorXd dq_out = Eigen::VectorXd::Zero(6);

    for (int n = 0; n < 30; ++n) {
        const Eigen::VectorXd q = ref_->random_q(rng_);
        const Eigen::VectorXd dq = test_utils::random_vector(rng_, 6, 1e-3);
        const Vector6 dx_in = test_utils::random_vector(rng_, 6, 1e-3);

        for (const std::string & link : ref_->link_names()) {
            SCOPED_TRACE("link " + link);
            bool ok[5] = {false, false, false, false, false};

            EXPECT_FALSE(allocates([&] {ok[0] = core_.calculate_link_transform(q, link, T);}));
            EXPECT_FALSE(allocates([&] {ok[1] = core_.calculate_jacobian(q, link, J);}));
            EXPECT_FALSE(allocates([&] {ok[2] = core_.calculate_jacobian_inverse(q, link, Ji);}));
            EXPECT_FALSE(allocates([&] {
                    ok[3] = core_.convert_joint_deltas_to_cartesian_deltas(q, dq, link, dx);
                }));
            EXPECT_FALSE(allocates([&] {
                    ok[4] = core_.convert_cartesian_deltas_to_joint_deltas(q, dx_in, link, dq_out);
                }));

            for (const bool r : ok) {
                EXPECT_TRUE(r) << "the call did not run through";
            }
        }
    }
}

TEST_F(MallocTest, TheGuardTripsWhenAnOutputHasToBeResized)
{
    // negative controls: an unsized output makes Eigen allocate, and the guard must notice
    const Eigen::VectorXd q = ref_->random_q(rng_);
    const Vector6 dx_in = test_utils::random_vector(rng_, 6, 1e-3);
    const std::string tcp = core_.tcp_link_name_;

    Eigen::VectorXd dq_unsized;
    EXPECT_TRUE(allocates([&] {core_.convert_cartesian_deltas_to_joint_deltas(q, dx_in, tcp, dq_unsized);}));

    Jacobian J_unsized;
    EXPECT_TRUE(allocates([&] {core_.calculate_jacobian(q, tcp, J_unsized);}));

    JacobianInverse Ji_unsized;
    EXPECT_TRUE(allocates([&] {core_.calculate_jacobian_inverse(q, tcp, Ji_unsized);}));

    // and the guard is switched off again afterwards
    Eigen::VectorXd allowed = Eigen::VectorXd::Zero(6);
    EXPECT_EQ(allowed.size(), 6);
}

TEST_F(MallocTest, ATemporaryFromAPlainProductAssignmentIsCaught)
{
    // Why the plugin uses .noalias(): "y = A * x" builds a temporary, which is a heap allocation
    // for a dynamic-size result. This pins down the reason for the .noalias() calls.
    const JacobianInverse A = JacobianInverse::Random(6, 6);
    const Vector6 x = Vector6::Random();
    Eigen::VectorXd y = Eigen::VectorXd::Zero(6);
    EXPECT_TRUE(allocates([&] {y = A * x;}));
    EXPECT_FALSE(allocates([&] {y.noalias() = A * x;}));
}

}  // namespace

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
