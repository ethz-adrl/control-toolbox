/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "ct/rbd/state/JointState.h"

#include <gtest/gtest.h>

using namespace ct::rbd;

const size_t nJoints = 4;
JointState<nJoints> js;
const double tolerance = 1e-3;

// define the test upper and lower limits
std::vector<double> lowerLimitVector(nJoints, -1.0);
std::vector<double> upperLimitVector(nJoints, 1.0);
Eigen::Matrix<double, nJoints, 1> lowerLimitEigen(lowerLimitVector.data());
Eigen::Matrix<double, nJoints, 1> upperLimitEigen(upperLimitVector.data());

TEST(JointStateTest, jointLimitTest)
{
    // CASE 1: test joint state respects both position and velocity limits
    js.setZero();
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitVector, upperLimitVector));
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen));
    ASSERT_TRUE(js.checkVelocityLimits(upperLimitVector));
    ASSERT_TRUE(js.checkVelocityLimits(upperLimitEigen));

    // CASE 2: test joint state violates an upper position limit
    js.getPosition(1) = 2.0;
    ASSERT_FALSE(js.checkPositionLimits(lowerLimitVector, upperLimitVector));
    ASSERT_FALSE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen));

    // CASE 3: test joint state violates a lower position limit
    js.getPosition(1) = -2.0;
    ASSERT_FALSE(js.checkPositionLimits(lowerLimitVector, upperLimitVector));
    ASSERT_FALSE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen));

    // CASE 4: test joint state violates an upper velocity limit
    js.setZero();
    js.getVelocity(1) = 2.0;
    ASSERT_FALSE(js.checkVelocityLimits(upperLimitVector));
    ASSERT_FALSE(js.checkVelocityLimits(upperLimitEigen));

    // CASE 5: test joint state violates a lower velocity limit
    js.getVelocity(1) = -2.0;
    ASSERT_FALSE(js.checkVelocityLimits(upperLimitVector));
    ASSERT_FALSE(js.checkVelocityLimits(upperLimitEigen));

    // CASE 6: test joint state violates an upper position limit within a custom tolerance
    js.setZero();
    js.getPosition(1) = upperLimitEigen(1) + 0.5 * tolerance;
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitVector, upperLimitVector, tolerance));
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen, tolerance));

    // CASE 7: test joint state violates a lower position limit within a custom tolerance
    js.getPosition(1) = lowerLimitEigen(1) - 0.5 * tolerance;
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitVector, upperLimitVector, tolerance));
    ASSERT_TRUE(js.checkPositionLimits(lowerLimitEigen, upperLimitEigen, tolerance));
}

TEST(JointStateTest, toUniqueTest)
{
    // CASE 1: no changes occur when joint position is in original range
    js.setZero();
    js.getPosition(1) = 0.1;
    JointState<nJoints> jsRef = js;
    js.toUniquePosition(lowerLimitVector);
    ASSERT_TRUE(js.isApprox(jsRef));

    // CASE 2: test normalization of a joint angle that is wrapped by 2*M_PI
    js.setZero();
    jsRef.setZero();
    jsRef.getPosition(1) = 0.1;
    js.getPosition(1) = 0.1 - 2 * M_PI;
    js.toUniquePosition(lowerLimitVector);
    ASSERT_TRUE(js.isApprox(jsRef));

    // CASE 3: test border tolerance upper (angle should remain unchanged)
    js.setZero();
    js.getPosition(1) = upperLimitEigen(1) + 0.5 * tolerance;
    jsRef = js;
    js.toUniquePosition(lowerLimitVector, tolerance);
    ASSERT_TRUE(js.isApprox(jsRef));

    // CASE 4: test border tolerance lower (angle should remain unchanged)
    js.setZero();
    js.getPosition(1) = lowerLimitEigen(1) - 0.5 * tolerance;
    jsRef = js;
    js.toUniquePosition(lowerLimitVector, tolerance);
    ASSERT_TRUE(js.isApprox(jsRef));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
