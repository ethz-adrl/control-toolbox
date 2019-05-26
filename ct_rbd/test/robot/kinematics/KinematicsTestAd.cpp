/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
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
    typedef float scalar_type;
    typedef TestHyQ::tpl::Kinematics<scalar_type> KinTpl_t;
    KinTpl_t kynTpl;

    JointState<TestHyQ::tpl::Kinematics<scalar_type>::NJOINTS, scalar_type> hyqJointState;
    tpl::RigidBodyPose<scalar_type> hyqPose;
    Eigen::Matrix<scalar_type, 3, 1> vec3Tpl;

    hyqPose.setRandom();
    vec3Tpl.setRandom();

    size_t ind = 1;

    Eigen::Matrix<scalar_type, 3, 1> pos = kynTpl.getEEPositionInWorld(ind, hyqPose, hyqJointState.getPositions());
    auto force3d = kynTpl.mapForceFromWorldToLink3d(vec3Tpl, hyqPose, hyqJointState.getPositions(), ind);
    auto forceLink = kynTpl.mapForceFromWorldToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);
    auto forceEELink = kynTpl.mapForceFromEEToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);

    // TODO: create test case for these float-quantities.

    ASSERT_TRUE(true);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
