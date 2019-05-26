/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <gtest/gtest.h>

#include <ct/rbd/state/JointState.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"
#include "../../../include/ct/rbd/robot/Kinematics.h"

#include "ct/rbd/robot/kinematics/EndEffector.h"

using namespace ct;
using namespace rbd;


TEST(TestHyQKinematics, transformTest)
{
    TestHyQ::Kinematics kyn;
    EndEffector<TestHyQ::Kinematics::NJOINTS> eeTest;

    for (size_t i = 0; i < TestHyQ::Kinematics::NUM_EE; i++)
    {
        eeTest = kyn.getEndEffector(i);
    }

    JointState<TestHyQ::Kinematics::NJOINTS> hyqJointState;
    Eigen::Matrix<double, 3, 1> vec3;

    RigidBodyPose hyqPose;

    hyqJointState.setRandom();
    hyqPose.setRandom();

    size_t ind = 1;

    Eigen::Vector3d pos = kyn.getEEPositionInWorld(ind, hyqPose, hyqJointState.getPositions());
    auto force3d = kyn.mapForceFromWorldToLink3d(vec3, hyqPose, hyqJointState.getPositions(), ind);
    auto forceLink = kyn.mapForceFromWorldToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);
    auto forceEELink = kyn.mapForceFromEEToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);

    // TODO: create test case for these float-quantities.

    ASSERT_TRUE(true);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
