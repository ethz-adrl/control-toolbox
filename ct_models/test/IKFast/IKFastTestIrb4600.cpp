/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/models/Irb4600/Irb4600InverseKinematics.h>
#include <ct/rbd/rbd.h>
#include <ct/rbd/state/JointState.h>

#include <gtest/gtest.h>

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

        typename ct::rbd::Irb4600InverseKinematics<double>::JointPositionsVector_t solutions;
        irb4600_ik_solver.computeInverseKinematics(solutions, ee_pose);

        for (const auto& joints : solutions)
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
