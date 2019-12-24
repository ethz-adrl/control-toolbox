/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-variable"

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
    typedef float size_type;
    typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;
    KinTpl_t kynTpl;

    JointState<TestHyQ::tpl::Kinematics<size_type>::NJOINTS, size_type> hyqJointState;
    tpl::RigidBodyPose<size_type> hyqPose;
    Eigen::Matrix<size_type, 3, 1> vec3Tpl;

    hyqPose.setRandom();
    vec3Tpl.setRandom();

    size_t ind = 1;

    kindr::Position<size_type, 3> pos = kynTpl.getEEPositionInWorld(ind, hyqPose, hyqJointState.getPositions());
    auto force3d = kynTpl.mapForceFromWorldToLink3d(vec3Tpl, hyqPose, hyqJointState.getPositions(), ind);
    auto forceLink = kynTpl.mapForceFromWorldToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);
    auto forceEELink = kynTpl.mapForceFromEEToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#pragma GCC diagnostic pop
