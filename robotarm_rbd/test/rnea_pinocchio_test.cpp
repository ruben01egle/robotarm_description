// Inverse dynamics (and kinematics) of RobotarmRbd on the real robot, compared with Pinocchio.
//
// Pinocchio is an independent implementation (spatial algebra, its own URDF parser), so agreement
// to ~1e-9 pins down frames, inertia parsing, the gravity handling and the TCP wrench convention at
// once. Pinocchio is a test-only dependency. Linked without rclcpp, like robotarm_rbd_test.
//
// Model correspondence: Pinocchio builds joints universe(0), axis1(1) .. axis6(6). Pinocchio joint j
// is our joints_[j-1]. It merges the inertia of links on fixed joints into their parent: the root link
// Base_1 goes to the universe (irrelevant for a fixed base, as for us), and flange, tool and tcp go to
// joint 6, which is exactly what our parser folds into the last moving link.
//
// Every test runs twice: on the real robot as it is ("none", tcp on the flange) and with a gripper
// that has mass between flange and tcp ("tool", see real_urdf_with_test_tool).

#include <gtest/gtest.h>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "real_urdf.hpp"
#include "testable_rbd.hpp"

namespace
{

using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

const Eigen::Vector3d default_gravity(0.0, 0.0, -9.81);

class RneaVsPinocchio : public ::testing::TestWithParam<std::string>
{
protected:
    void SetUp() override
    {
        std::string urdf;
        try {
            urdf = GetParam() == "tool" ? test_utils::real_urdf_with_test_tool() : test_utils::real_urdf();
        } catch (const std::exception & e) {
            FAIL() << e.what();
        }
        ASSERT_TRUE(rbd_.initialize(urdf, {})) << rbd_.last_error();
        ASSERT_TRUE(rbd_.get_joint_names(joint_names_));
        ASSERT_TRUE(rbd_.get_joint_limits(limits_));
        ASSERT_TRUE(rbd_.get_link_names(link_names_));
        ASSERT_TRUE(rbd_.get_tcp_link_name(tcp_name_));
        n_ = static_cast<Eigen::Index>(joint_names_.size());

        pinocchio::urdf::buildModelFromXML(urdf, model_);
        data_ = pinocchio::Data(model_);
        rng_.seed(4711);
    }

    Eigen::VectorXd random_q()
    {
        Eigen::VectorXd q(n_);
        for (Eigen::Index i = 0; i < n_; ++i) {
            const auto & l = limits_[static_cast<size_t>(i)];
            q[i] = std::uniform_real_distribution<double>(l.min, l.max)(rng_);
        }
        return q;
    }

    Eigen::VectorXd random_vector(double scale)
    {
        std::uniform_real_distribution<double> u(-scale, scale);
        Eigen::VectorXd v(n_);
        for (Eigen::Index i = 0; i < n_; ++i) {
            v[i] = u(rng_);
        }
        return v;
    }

    Eigen::Vector3d random_vector3(double scale)
    {
        std::uniform_real_distribution<double> u(-scale, scale);
        return Eigen::Vector3d(u(rng_), u(rng_), u(rng_));
    }

    // q = 0, the pose with the forearm pointing up (axis2 = -pi/2), and random poses inside the limits
    std::vector<Eigen::VectorXd> static_poses()
    {
        std::vector<Eigen::VectorXd> poses;
        poses.push_back(Eigen::VectorXd::Zero(n_));
        Eigen::VectorXd up = Eigen::VectorXd::Zero(n_);
        up[1] = -M_PI / 2;
        poses.push_back(up);
        for (int k = 0; k < 20; ++k) {
            poses.push_back(random_q());
        }
        return poses;
    }

    Eigen::VectorXd our_tau(
        const Eigen::VectorXd & q, const Eigen::VectorXd & dq, const Eigen::VectorXd & ddq,
        const Eigen::Vector3d & F_tcp = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d & M_tcp = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d & gravity = default_gravity)
    {
        Eigen::VectorXd tau(n_);
        EXPECT_TRUE(rbd_.recursive_newton_euler(q, dq, ddq, tau, F_tcp, M_tcp, gravity))
            << rbd_.last_error();
        return tau;
    }

    test_utils::TestableRbd rbd_;
    std::vector<std::string> joint_names_, link_names_;
    std::vector<robotarm_rbd::RobotarmRbd::Limits> limits_;
    std::string tcp_name_;
    Eigen::Index n_ = 0;

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::mt19937 rng_;
};

INSTANTIATE_TEST_SUITE_P(
    Tools, RneaVsPinocchio, ::testing::Values("none", "tool"),
    [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

}  // namespace

TEST_P(RneaVsPinocchio, BothModelsHaveTheSameJointsAndMasses)
{
    ASSERT_EQ(model_.nq, n_);
    ASSERT_EQ(model_.nv, n_);
    ASSERT_EQ(model_.njoints, n_ + 1);  // + universe
    double our_mass = 0.0;
    for (Eigen::Index i = 0; i < n_; ++i) {
        const size_t j = static_cast<size_t>(i) + 1;
        EXPECT_EQ(model_.names[j], joint_names_[static_cast<size_t>(i)]) << "joint order";
        // mass, CoM and tensor about the CoM, all in the joint frame: for the last link that is
        // the folded tool against Pinocchio's own merge
        const auto & ours = rbd_.joints_[static_cast<size_t>(i)].inertia_;
        EXPECT_NEAR(model_.inertias[j].mass(), ours.mass, 1e-12);
        EXPECT_LT((model_.inertias[j].lever() - ours.com).norm(), 1e-12);
        EXPECT_LT((model_.inertias[j].inertia().matrix() - ours.com_inertia_in_joint).cwiseAbs().maxCoeff(), 1e-12)
            << "ours\n" << ours.com_inertia_in_joint << "\npin\n" << model_.inertias[j].inertia().matrix();
        our_mass += ours.mass;
    }
    EXPECT_GT(our_mass, 1.0) << "the real robot must come with its inertias";
}

TEST_P(RneaVsPinocchio, TheToolMassIsFoldedIntoTheLastLink)
{
    // premise of the "tool" runs: the gripper really adds its 0.8 kg to the last link
    test_utils::TestableRbd plain;
    ASSERT_TRUE(plain.initialize(test_utils::real_urdf(), {})) << plain.last_error();
    const double added = rbd_.joints_.back().inertia_.mass - plain.joints_.back().inertia_.mass;
    EXPECT_NEAR(added, GetParam() == "tool" ? 0.8 : 0.0, 1e-12);
    for (size_t i = 0; i + 1 < rbd_.joints_.size(); ++i) {
        EXPECT_EQ(rbd_.joints_[i].inertia_.mass, plain.joints_[i].inertia_.mass) << "only the last link";
    }
}

TEST_P(RneaVsPinocchio, StaticGravityTorques)
{
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n_);
    double max_err = 0.0;
    for (const Eigen::VectorXd & q : static_poses()) {
        const Eigen::VectorXd tau = our_tau(q, zero, zero);

        // axis1 is vertical with an identity origin: gravity has no moment about it
        EXPECT_NEAR(tau[0], 0.0, 1e-12);

        // By hand (principle of virtual work): the generalised gravity force is sum_k J_com,kᵀ m_k g,
        // the motor holds against it: tau = -sum_k m_k J_com,kᵀ g, with the linear Jacobian of the
        // CoM of link k, column j = z_j × (c_k - o_j) for j <= k. Built from our forward kinematics
        // only, no RNEA involved.
        Eigen::VectorXd tau_hand = Eigen::VectorXd::Zero(n_);
        std::vector<Eigen::Isometry3d> T(static_cast<size_t>(n_));
        for (Eigen::Index k = 0; k < n_; ++k) {
            ASSERT_TRUE(rbd_.calculate_link_transform(q, link_names_[static_cast<size_t>(k) + 1],
                T[static_cast<size_t>(k)]));
        }
        for (Eigen::Index k = 0; k < n_; ++k) {
            const auto & in = rbd_.joints_[static_cast<size_t>(k)].inertia_;
            const Eigen::Vector3d c = T[static_cast<size_t>(k)] * in.com;
            for (Eigen::Index j = 0; j <= k; ++j) {
                const Eigen::Isometry3d & Tj = T[static_cast<size_t>(j)];
                const Eigen::Vector3d z = Tj.linear() * rbd_.joints_[static_cast<size_t>(j)].joint_axis_in_child_;
                tau_hand[j] -= in.mass * z.cross(c - Tj.translation()).dot(default_gravity);
            }
        }
        EXPECT_LT((tau - tau_hand).cwiseAbs().maxCoeff(), 1e-10)
            << "ours " << tau.transpose() << "\nhand " << tau_hand.transpose();

        const Eigen::VectorXd tau_pin = pinocchio::rnea(model_, data_, q, zero, zero);
        EXPECT_LT((tau - tau_pin).cwiseAbs().maxCoeff(), 1e-10)
            << "ours " << tau.transpose() << "\npin  " << tau_pin.transpose();
        max_err = std::max(max_err, (tau - tau_pin).cwiseAbs().maxCoeff());
    }
    std::printf("static gravity: max |tau - tau_pinocchio| = %.3g Nm\n", max_err);
}

TEST_P(RneaVsPinocchio, MassMatrixFromUnitAccelerations)
{
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n_);
    const Eigen::Vector3d no_gravity = Eigen::Vector3d::Zero();
    double max_err = 0.0;
    for (int n = 0; n < 50; ++n) {
        const Eigen::VectorXd q = n == 0 ? zero : random_q();

        // g = 0, dq = 0: tau = M(q) ddq, so ddq = e_j gives column j
        Eigen::MatrixXd M(n_, n_);
        for (Eigen::Index j = 0; j < n_; ++j) {
            M.col(j) = our_tau(q, zero, Eigen::VectorXd::Unit(n_, j), Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero(), no_gravity);
        }

        EXPECT_LT((M - M.transpose()).cwiseAbs().maxCoeff(), 1e-12) << "M(q) must be symmetric";
        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(M);
        EXPECT_GT(eig.eigenvalues().minCoeff(), 0.0) << "M(q) must be positive definite";

        pinocchio::crba(model_, data_, q);
        Eigen::MatrixXd M_pin = data_.M;
        M_pin.triangularView<Eigen::StrictlyLower>() = M_pin.transpose().triangularView<Eigen::StrictlyLower>();
        EXPECT_LT((M - M_pin).cwiseAbs().maxCoeff(), 1e-10) << "ours\n" << M << "\npin\n" << M_pin;
        max_err = std::max(max_err, (M - M_pin).cwiseAbs().maxCoeff());
    }
    std::printf("mass matrix: max |M - M_crba| = %.3g kg m^2\n", max_err);
}

TEST_P(RneaVsPinocchio, FullDynamicsForRandomStates)
{
    double max_err = 0.0, max_tau = 0.0;
    for (int n = 0; n < 500; ++n) {
        const Eigen::VectorXd q = random_q();
        const Eigen::VectorXd dq = random_vector(2.0);
        const Eigen::VectorXd ddq = random_vector(5.0);

        const Eigen::VectorXd tau = our_tau(q, dq, ddq);
        const Eigen::VectorXd tau_pin = pinocchio::rnea(model_, data_, q, dq, ddq);
        const double err = (tau - tau_pin).cwiseAbs().maxCoeff();
        EXPECT_LT(err, 1e-9) << "q " << q.transpose() << "\ndq " << dq.transpose() << "\nddq "
                             << ddq.transpose() << "\nours " << tau.transpose() << "\npin  "
                             << tau_pin.transpose();
        max_err = std::max(max_err, err);
        max_tau = std::max(max_tau, tau_pin.cwiseAbs().maxCoeff());
    }
    std::printf("full dynamics: max |tau - tau_pinocchio| = %.3g Nm (max |tau| = %.3g Nm)\n",
        max_err, max_tau);
}

TEST_P(RneaVsPinocchio, TcpWrench)
{
    // Our F_tcp/M_tcp: exerted BY the robot ON the environment, at the tcp, in the tcp frame.
    // Pinocchio's fext[j]: exerted ON the body of joint j BY the environment, in the joint j frame,
    // about its origin. So: flip the sign (actio = reactio), then move the wrench from the tcp frame
    // to the frame of the parent joint with the tcp placement (rotation + lever arm p × f).
    const pinocchio::FrameIndex tcp_id = model_.getFrameId(tcp_name_);
    ASSERT_LT(tcp_id, model_.frames.size());
    const pinocchio::JointIndex parent = model_.frames[tcp_id].parentJoint;
    ASSERT_EQ(parent, static_cast<pinocchio::JointIndex>(n_)) << "tcp must hang on the last joint";
    const pinocchio::SE3 & tcp_in_joint = model_.frames[tcp_id].placement;

    double max_err = 0.0;
    for (int n = 0; n < 200; ++n) {
        const Eigen::VectorXd q = random_q();
        const Eigen::VectorXd dq = n % 2 ? random_vector(2.0) : Eigen::VectorXd::Zero(n_);
        const Eigen::VectorXd ddq = n % 2 ? random_vector(5.0) : Eigen::VectorXd::Zero(n_);
        const Eigen::Vector3d F = random_vector3(50.0);
        const Eigen::Vector3d M = random_vector3(5.0);

        std::vector<pinocchio::Force> fext(model_.joints.size(), pinocchio::Force::Zero());
        fext[parent] = tcp_in_joint.act(pinocchio::Force(-F, -M));

        const Eigen::VectorXd tau = our_tau(q, dq, ddq, F, M);
        const Eigen::VectorXd tau_pin = pinocchio::rnea(model_, data_, q, dq, ddq, fext);
        const double err = (tau - tau_pin).cwiseAbs().maxCoeff();
        EXPECT_LT(err, 1e-9) << "ours " << tau.transpose() << "\npin  " << tau_pin.transpose();
        max_err = std::max(max_err, err);
    }
    std::printf("tcp wrench: max |tau - tau_pinocchio| = %.3g Nm\n", max_err);

    // Without gravity and motion only the wrench is left: tau = Jᵀ w, with our Jacobian of the tcp
    // (base frame, reference point tcp) and the wrench turned into the base frame
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n_);
    for (int n = 0; n < 20; ++n) {
        const Eigen::VectorXd q = random_q();
        const Eigen::Vector3d F = random_vector3(50.0);
        const Eigen::Vector3d M = random_vector3(5.0);
        Eigen::Isometry3d T_tcp;
        Jacobian J(6, n_);
        ASSERT_TRUE(rbd_.calculate_link_transform(q, tcp_name_, T_tcp));
        ASSERT_TRUE(rbd_.calculate_jacobian(q, tcp_name_, J));
        Eigen::Matrix<double, 6, 1> w;
        w << T_tcp.linear() * F, T_tcp.linear() * M;
        const Eigen::VectorXd tau = our_tau(q, zero, zero, F, M, Eigen::Vector3d::Zero());
        EXPECT_LT((tau - J.transpose() * w).cwiseAbs().maxCoeff(), 1e-10);
    }
}

TEST_P(RneaVsPinocchio, ForwardKinematicsAndJacobian)
{
    std::vector<std::string> links = link_names_;
    links.push_back("flange");
    links.push_back(tcp_name_);
    for (int n = 0; n < 50; ++n) {
        const Eigen::VectorXd q = n == 0 ? Eigen::VectorXd::Zero(n_) : random_q();
        pinocchio::framesForwardKinematics(model_, data_, q);
        for (const std::string & link : links) {
            SCOPED_TRACE("link " + link);
            ASSERT_TRUE(model_.existFrame(link));
            const pinocchio::FrameIndex id = model_.getFrameId(link);

            Eigen::Isometry3d T;
            ASSERT_TRUE(rbd_.calculate_link_transform(q, link, T));
            EXPECT_LT((T.translation() - data_.oMf[id].translation()).norm(), 1e-12);
            EXPECT_LT((T.linear() - data_.oMf[id].rotation()).cwiseAbs().maxCoeff(), 1e-12);

            // both: base frame axes, reference point = origin of the link, linear rows first
            Jacobian J(6, n_);
            Jacobian J_pin = Jacobian::Zero(6, n_);
            ASSERT_TRUE(rbd_.calculate_jacobian(q, link, J));
            pinocchio::computeFrameJacobian(model_, data_, q, id, pinocchio::LOCAL_WORLD_ALIGNED, J_pin);
            EXPECT_LT((J - J_pin).cwiseAbs().maxCoeff(), 1e-12) << "ours\n" << J << "\npin\n" << J_pin;
        }
    }
}
