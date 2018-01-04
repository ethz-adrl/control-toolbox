/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "ct/rbd/state/JointState.h"

#include <gtest/gtest.h>

using namespace ct::rbd;

TEST(JointStateTest, jointLimitTest)
{
    JointState<4> js;
    std::vector<double> lowerLimitVector(4, -1), upperLimitVector(4, 1);
    Eigen::Vector4d lowerLimitEigen(lowerLimitVector.data()), upperLimitEigen(upperLimitVector.data());

    ASSERT_TRUE(js.checkPositionLimits(lowerLimitVector, upperLimitVector));
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen));

    js.getPosition(1) = 2;
    ASSERT_FALSE(js.checkPositionLimits(lowerLimitVector, upperLimitVector));
    ASSERT_FALSE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen));

    ASSERT_TRUE(js.checkVelocityLimits(upperLimitVector));
    ASSERT_TRUE(js.checkVelocityLimits(upperLimitEigen));

    js.getVelocity(1) = 2;
    ASSERT_FALSE(js.checkVelocityLimits(upperLimitVector));
    ASSERT_FALSE(js.checkVelocityLimits(upperLimitEigen));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
