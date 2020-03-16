/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <iostream>
#include <cstdlib>

#include <gtest/gtest.h>

#include <ct/core/core.h>


using namespace ct::core;


TEST(LinspaceTest, LinspaceTest)
{
    StateVector<2> start, end;
    start << 1, 5;
    end << 5, 1;

    size_t nPoints = 5;

    StateVectorArray<2> traj = linspace<StateVectorArray<2>>(start, end, nPoints);

    // the desired result is
    /*
	 * [1 2 3 4 5]
	 * [5 4 3 2 1]
	 * */
    for (size_t i = 0; i < nPoints; i++)
    {
        ASSERT_EQ(traj[i](0), i + 1);
        ASSERT_EQ(traj[i](1), 5 - i);
    }
}


/*!
 *  \example LinspaceTest.cpp
 *
 *  This is a trivial test for the linspace-class
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
