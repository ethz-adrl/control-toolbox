/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <gtest/gtest.h>

#include "ct/rbd/state/JointState.h"
#include "../../models/testhyq/RobCoGenTestHyQ.h"

TEST(RobCoGenContainerTestHyQ, accessorTest)
{
    ct::rbd::TestHyQ::RobCoGenContainer testHyQModel;

    const size_t njoints = ct::rbd::TestHyQ::RobCoGenContainer::NJOINTS;

    ct::rbd::JointState<njoints>::Position jointPos;
    jointPos.setZero();

    // call random functions
    testHyQModel.jSim().update(jointPos);
    testHyQModel.jSim().computeL();

    testHyQModel.inertiaProperties().getTotalMass();

    testHyQModel.homogeneousTransforms().fr_LH_foot_X_fr_trunk(jointPos);
    testHyQModel.jacobians().fr_trunk_J_LF_hipassemblyCOM(jointPos);
    testHyQModel.inertiaProperties().getCOM_trunk();
}

TEST(RobCoGenContainerTestHyQ, transformTest)
{
    ct::rbd::TestHyQ::RobCoGenContainer testHyQModel;
    auto& robcogenTrans = testHyQModel.homogeneousTransforms();

    const size_t njoints = ct::rbd::TestHyQ::RobCoGenContainer::NJOINTS;

    ct::rbd::JointState<njoints>::Position jointPos;
    jointPos.setRandom();

    typedef ct::rbd::TestHyQ::Kinematics::HomogeneousTransform transform;

    transform Base_LF_hipassembly = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 0, jointPos);
    transform Base_LF_upperleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 1, jointPos);
    transform Base_LF_lowerleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 2, jointPos);
    transform Base_RF_hipassembly = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 3, jointPos);
    transform Base_RF_upperleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 4, jointPos);
    transform Base_RF_lowerleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 5, jointPos);
    transform Base_LH_hipassembly = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 6, jointPos);
    transform Base_LH_upperleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 7, jointPos);
    transform Base_LH_lowerleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 8, jointPos);
    transform Base_RH_hipassembly = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 9, jointPos);
    transform Base_RH_upperleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 10, jointPos);
    transform Base_RH_lowerleg = testHyQModel.getHomogeneousTransformBaseLinkById(1 + 11, jointPos);

    ASSERT_TRUE(Base_LF_hipassembly == robcogenTrans.fr_trunk_X_fr_LF_hipassembly(jointPos));
    ASSERT_TRUE(Base_LF_upperleg == robcogenTrans.fr_trunk_X_fr_LF_upperleg(jointPos));
    ASSERT_TRUE(Base_LF_lowerleg == robcogenTrans.fr_trunk_X_fr_LF_lowerleg(jointPos));
    ASSERT_TRUE(Base_RF_hipassembly == robcogenTrans.fr_trunk_X_fr_RF_hipassembly(jointPos));
    ASSERT_TRUE(Base_RF_upperleg == robcogenTrans.fr_trunk_X_fr_RF_upperleg(jointPos));
    ASSERT_TRUE(Base_RF_lowerleg == robcogenTrans.fr_trunk_X_fr_RF_lowerleg(jointPos));
    ASSERT_TRUE(Base_LH_hipassembly == robcogenTrans.fr_trunk_X_fr_LH_hipassembly(jointPos));
    ASSERT_TRUE(Base_LH_upperleg == robcogenTrans.fr_trunk_X_fr_LH_upperleg(jointPos));
    ASSERT_TRUE(Base_LH_lowerleg == robcogenTrans.fr_trunk_X_fr_LH_lowerleg(jointPos));
    ASSERT_TRUE(Base_RH_hipassembly == robcogenTrans.fr_trunk_X_fr_RH_hipassembly(jointPos));
    ASSERT_TRUE(Base_RH_upperleg == robcogenTrans.fr_trunk_X_fr_RH_upperleg(jointPos));
    ASSERT_TRUE(Base_RH_lowerleg == robcogenTrans.fr_trunk_X_fr_RH_lowerleg(jointPos));

    // test a false one
    ASSERT_FALSE(Base_RH_lowerleg == Base_LF_hipassembly);

    // test for zero
    ASSERT_FALSE(Base_RH_lowerleg.isApprox(transform::Zero()));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
