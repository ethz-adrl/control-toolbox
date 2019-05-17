/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/models/HyA/HyAInverseKinematics.h>
#include <ct/rbd/rbd.h>
#include <ct/rbd/state/JointState.h>
#include <ct/models/HyA/HyA.h>

#include <gtest/gtest.h>

TEST(HyAIKTest, FKTest)
{
    ct::rbd::HyA::Kinematics kin;
    typename ct::rbd::JointState<ct::rbd::HyA::Kinematics::NJOINTS, double>::Position pos;
    for (int i = 0; i < 10; ++i)
    {
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
    for (int i = 0; i < 100; ++i)
    {
        pos.setRandom();

        auto ee_pose = kin.getEEPoseInBase(0, pos);

        ct::rbd::RigidBodyPose basePose;
        basePose.setIdentity();

        typename ct::rbd::HyAInverseKinematics<double>::JointPositionsVector_t solutions_base;
        typename ct::rbd::HyAInverseKinematics<double>::JointPositionsVector_t solutions_world;

        bool solutionFound_base = hya_ik_solver.computeInverseKinematics(solutions_base, ee_pose);
        bool solutionFound_world = hya_ik_solver.computeInverseKinematics(solutions_world, ee_pose, basePose);
        ASSERT_TRUE(solutionFound_base == solutionFound_world);

        if (solutionFound_base)
        {
            for (const auto& joints : solutions_base)
            {
                auto query_ee_pose = kin.getEEPoseInBase(0, joints);
                ASSERT_LT(
                    (query_ee_pose.position().toImplementation() - ee_pose.position().toImplementation()).norm(), 1e-6);
                ASSERT_LT((query_ee_pose.getRotationMatrix().toImplementation() -
                              ee_pose.getRotationMatrix().toImplementation())
                              .norm(),
                    1e-6);
            }
        }
    }
}

// Test close-to functionality of HyA inverse kinematics
TEST(HyAIKTest, IKFastTest_closeTo)
{
    ct::rbd::HyAInverseKinematics<double> hya_ik_solver;
    ct::rbd::HyA::Kinematics kin;
    typename ct::rbd::JointState<ct::rbd::HyA::Kinematics::NJOINTS, double>::Position pos;
    for (int i = 0; i < 100; ++i)
    {
        pos.setRandom();

        auto ee_pose = kin.getEEPoseInBase(0, pos);

        ct::rbd::RigidBodyPose basePose;
        basePose.setIdentity();

        typename ct::rbd::HyAInverseKinematics<double>::JointPosition_t ref_position;
        ref_position.setRandom();
        typename ct::rbd::HyAInverseKinematics<double>::JointPosition_t solution_base;
        typename ct::rbd::HyAInverseKinematics<double>::JointPosition_t solution_world;

        bool solutionFound_base = hya_ik_solver.computeInverseKinematicsCloseTo(solution_base, ee_pose, ref_position);
        bool solutionFound_world = hya_ik_solver.computeInverseKinematicsCloseTo(solution_world, ee_pose, basePose, ref_position);
        ASSERT_TRUE(solutionFound_base == solutionFound_world);

        if (solutionFound_base)
        {
        	ASSERT_LT((solution_base-solution_world).array().abs().maxCoeff(), 1e-6);
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
