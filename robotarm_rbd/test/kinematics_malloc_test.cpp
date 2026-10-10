// The real-time path of Kinematics and RobotarmRbd (incl. the inverse dynamics) must not allocate when
// the caller passes pre-sized outputs.
//
// Eigen can turn heap allocations into failed assertions (EIGEN_RUNTIME_NO_MALLOC, see
// eigen_malloc_guard.hpp). The guard has to be compiled into the code under test, so this target
// compiles src/Kinematics.cpp itself instead of linking the library.
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

    test_utils::TestableKinematics core_;
    std::string urdf_;
    std::unique_ptr<test_utils::ReferenceFk> ref_;
    std::mt19937 rng_;
};

TEST_F(MallocTest, PresizedCallsDoNotAllocateForAnyLink)
{
    // both inverse methods, and besides the 6 joint robot chains with fewer and more joints: the
    // svd sizes its buffers by min(6, n), which only differs from n for n != 6
    struct Robot
    {
        std::string name;
        std::string urdf;
    };
    const std::vector<Robot> robots{
        {"baseline", urdf_},
        {"random3", test_utils::make_random_spec(3, 103).str()},
        {"random7", test_utils::make_random_spec(7, 107).str()}};

    for (const Robot & robot : robots) {
        const test_utils::ReferenceFk ref(robot.urdf);
        const auto n = static_cast<Eigen::Index>(ref.dof());
        for (const std::string method : {"ldlt", "svd"}) {
            SCOPED_TRACE(robot.name + ", " + method);
            test_utils::TestableKinematics core;
            ASSERT_TRUE(test_utils::initialize(core, robot.urdf, test_utils::inverse_params(0.05, method)));

            Eigen::Isometry3d T;
            Jacobian J = Jacobian::Zero(6, n);
            JacobianInverse Ji = JacobianInverse::Zero(n, 6);
            Vector6 dx = Vector6::Zero();
            Eigen::VectorXd dq_out = Eigen::VectorXd::Zero(n);

            for (int k = 0; k < 10; ++k) {
                const Eigen::VectorXd q = ref.random_q(rng_);
                const Eigen::VectorXd dq = test_utils::random_vector(rng_, n, 1e-3);
                const Vector6 dx_in = test_utils::random_vector(rng_, 6, 1e-3);

                for (const std::string & link : ref.link_names()) {
                    SCOPED_TRACE("link " + link);
                    bool ok[5] = {false, false, false, false, false};

                    EXPECT_FALSE(allocates([&] {ok[0] = core.calculate_link_transform(q, link, T);}));
                    EXPECT_FALSE(allocates([&] {ok[1] = core.calculate_jacobian(q, link, J);}));
                    EXPECT_FALSE(allocates([&] {ok[2] = core.calculate_jacobian_inverse(q, link, Ji);}));
                    EXPECT_FALSE(allocates([&] {
                            ok[3] = core.convert_joint_deltas_to_cartesian_deltas(q, dq, link, dx);
                        }));
                    EXPECT_FALSE(allocates([&] {
                            ok[4] = core.convert_cartesian_deltas_to_joint_deltas(q, dx_in, link, dq_out);
                        }));

                    for (const bool r : ok) {
                        EXPECT_TRUE(r) << "the call did not run through";
                    }
                }
            }
        }
    }
}

TEST(MallocTestRbd, PresizedSingularityInfoDoesNotAllocate)
{
    // the SingularityInfo output of the svd inverse, sized with SingularityInfo(dof), for chains
    // with fewer, exactly and more than 6 joints
    for (const size_t dof : {3u, 6u, 7u}) {
        SCOPED_TRACE("dof = " + std::to_string(dof));
        const std::string urdf = test_utils::make_random_spec(dof, 100 + static_cast<unsigned>(dof)).str();
        const test_utils::ReferenceFk ref(urdf);
        robotarm_rbd::RobotarmRbd rbd;
        robotarm_rbd::RobotarmRbd::Config config;
        config.jinv_method = robotarm_rbd::RobotarmRbd::JinvMethod::SVD;
        ASSERT_TRUE(rbd.initialize(urdf, config)) << rbd.last_error();
        std::string tcp;
        ASSERT_TRUE(rbd.get_tcp_link_name(tcp));

        const auto n = static_cast<Eigen::Index>(dof);
        JacobianInverse Ji = JacobianInverse::Zero(n, 6);
        robotarm_rbd::RobotarmRbd::SingularityInfo info(n);
        std::mt19937 rng(4711);
        for (int k = 0; k < 10; ++k) {
            const Eigen::VectorXd q = ref.random_q(rng);
            bool ok = false;
            EXPECT_FALSE(allocates([&] {ok = rbd.calculate_jinv_svd(q, tcp, Ji, &info);}));
            EXPECT_TRUE(ok) << rbd.last_error();
        }

        // negative control: an unsized SingularityInfo has to be allocated
        robotarm_rbd::RobotarmRbd::SingularityInfo info_unsized;
        const Eigen::VectorXd q = ref.random_q(rng);
        EXPECT_TRUE(allocates([&] {rbd.calculate_jinv_svd(q, tcp, Ji, &info_unsized);}));
    }
}

TEST_F(MallocTest, TheGuardTripsWhenAnOutputHasToBeResized)
{
    // negative controls: an unsized output makes Eigen allocate, and the guard must notice
    const Eigen::VectorXd q = ref_->random_q(rng_);
    const Vector6 dx_in = test_utils::random_vector(rng_, 6, 1e-3);
    const std::string tcp = test_utils::tcp_name(core_);

    Eigen::VectorXd dq_unsized;
    EXPECT_TRUE(allocates([&] {core_.convert_cartesian_deltas_to_joint_deltas(q, dx_in, tcp, dq_unsized);}));

    Jacobian J_unsized;
    EXPECT_TRUE(allocates([&] {core_.calculate_jacobian(q, tcp, J_unsized);}));

    JacobianInverse Ji_unsized;
    EXPECT_TRUE(allocates([&] {core_.calculate_jacobian_inverse(q, tcp, Ji_unsized);}));

    // the same for the ldlt inverse (core_ runs the default svd)
    test_utils::TestableKinematics ldlt;
    ASSERT_TRUE(test_utils::initialize(ldlt, urdf_, test_utils::inverse_params(0.01, "ldlt")));
    JacobianInverse Ji_unsized_ldlt;
    EXPECT_TRUE(allocates([&] {ldlt.calculate_jacobian_inverse(q, tcp, Ji_unsized_ldlt);}));

    // and the guard is switched off again afterwards
    Eigen::VectorXd allowed = Eigen::VectorXd::Zero(6);
    EXPECT_EQ(allowed.size(), 6);
}

TEST(MallocTestRnea, PresizedInverseDynamicsDoesNotAllocate)
{
    // the synthetic URDF of MallocTest has no inertias, the real robot has
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(test_utils::real_urdf(), {})) << rbd.last_error();
    std::mt19937 rng(4711);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);
    const Eigen::Vector3d F(1, 2, 3), M(0.1, 0.2, 0.3);

    for (int n = 0; n < 30; ++n) {
        const Eigen::VectorXd q = test_utils::random_vector(rng, 6, 1.0);
        const Eigen::VectorXd dq = test_utils::random_vector(rng, 6, 2.0);
        const Eigen::VectorXd ddq = test_utils::random_vector(rng, 6, 5.0);
        bool ok = false;
        EXPECT_FALSE(allocates([&] {ok = rbd.recursive_newton_euler(q, dq, ddq, tau, F, M);}));
        EXPECT_TRUE(ok) << rbd.last_error();
    }

    // negative control: an unsized tau has to be allocated
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd tau_unsized;
    EXPECT_TRUE(allocates([&] {rbd.recursive_newton_euler(zero, zero, zero, tau_unsized);}));
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
