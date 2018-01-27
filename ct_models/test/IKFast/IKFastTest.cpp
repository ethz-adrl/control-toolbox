/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/models/HyA/HyAInverseKinematics.h>
#include <ct/models/Irb4600/Irb4600InverseKinematics.h>
#include <ct/rbd/rbd.h>
#include <ct/rbd/state/JointState.h>
#include <ct/models/HyA/HyA.h>

#include <gtest/gtest.h>

TEST(HyAIKTest, FKTest)
{
    ct::rbd::HyAInverseKinematics<double> hya_ik_solver;
    ct::rbd::HyA::Kinematics kin;
    typename ct::rbd::JointState<ct::rbd::HyA::Kinematics::NJOINTS, double>::Position pos;
    for (int i = 0; i < 10; ++i) {
        pos.setRandom();

        auto ee_pose = kin.getEEPoseInBase(0, pos);

        Eigen::Vector3d ee_pos;
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> ee_rot;
        // Data needs to be in row-major form.
        hya_ik::ComputeFk(pos.data(), ee_pos.data(), ee_rot.data());

        ASSERT_LT((ee_pos - ee_pose.position().toImplementation()).norm(), 1e-6);
        ASSERT_LT((ee_rot - ee_pose.getRotationMatrix().toImplementation()).norm(), 1e-6);
    }
}

TEST(HyAIKTest, IKFastTest)
{
    ct::rbd::HyAInverseKinematics<double> hya_ik_solver;
    ct::rbd::HyA::Kinematics kin;
    typename ct::rbd::JointState<ct::rbd::HyA::Kinematics::NJOINTS, double>::Position pos;
    for (int i = 0; i < 100; ++i) {
        pos.setRandom();

        auto ee_pose = kin.getEEPoseInBase(0, pos);

        std::vector<typename ct::rbd::JointState<ct::rbd::HyA::Kinematics::NJOINTS, double>::Position> solutions =
            hya_ik_solver.computeInverseKinematics(ee_pose);
        std::cerr << "Found " << solutions.size() << " valid solutions." << std::endl;
        for (const auto& joints : solutions)
        {
            auto query_ee_pose = kin.getEEPoseInBase(0, joints);
            ASSERT_LT((query_ee_pose.position().toImplementation() - ee_pose.position().toImplementation()).norm(), 1e-6);
            ASSERT_LT(
                (query_ee_pose.getRotationMatrix().toImplementation() - ee_pose.getRotationMatrix().toImplementation())
                    .norm(),
                1e-6);
        }
    }
}

TEST(Irb4600IKTest, IKFastTest)
{
    ct::rbd::Irb4600InverseKinematics<double> irb4600_ik_solver;
    typename ct::rbd::JointState<6, double>::Position pos;
    for (int i = 0; i < 10; ++i)
    {
        pos.setRandom();

        Eigen::Vector3d ee_pos;
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> ee_rot;
        // Data needs to be in row-major form.
        irb4600_ik::ComputeFk(pos.data(), ee_pos.data(), ee_rot.data());

        ct::rbd::RigidBodyPose ee_pose;
        ee_pose.position().toImplementation() = ee_pos;
        ee_pose.setFromRotationMatrix(kindr::RotationMatrix<double>(ee_rot));

        for (const auto& joints : irb4600_ik_solver.computeInverseKinematics(ee_pose))
        {
            irb4600_ik::ComputeFk(joints.data(), ee_pos.data(), ee_rot.data());
            ASSERT_LT((ee_pos - ee_pose.position().toImplementation()).norm(), 1e-3);
            ASSERT_LT((ee_rot - ee_pose.getRotationMatrix().toImplementation()).norm(), 1e-3);
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
