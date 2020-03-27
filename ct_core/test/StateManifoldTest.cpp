/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>

#include <gtest/gtest.h>

using namespace ct::core;

TEST(StateManifoldTest, StateManifoldTest)
{
    using State_t = ManifoldState<manif::SE3<double>, manif::SE3Tangent<double>>;
    State_t se3;
    ASSERT_TRUE(State_t::TangentDim == 6);

    using EuclideanState_t = StateVector<6>;
    EuclideanState_t R6;
    ASSERT_TRUE(EuclideanState_t::TangentDim == 6);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
