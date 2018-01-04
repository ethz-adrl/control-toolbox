/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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

    RigidBodyPose hyqPose;

    hyqJointState.setRandom();
    hyqPose.setRandom();

    size_t ind = 1;

    kindr::Position3D pos = kyn.getEEPositionInWorld(ind, hyqPose, hyqJointState.getPositions());
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
