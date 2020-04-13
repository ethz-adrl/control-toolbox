/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <iostream>
#include <cstdlib>

#include <ct/core/core.h>

#include <gtest/gtest.h>

using namespace ct::core;


/**
 * this implements a trivial test for the trajectory class
 */
TEST(TrajectoryTest, EuclideanTest)
{
    // a trajectory with 10 points, starting at t = 0 with dt = 1.0
    ct::core::Time dt = 1.0;
    size_t nPoints = 10;

    TimeArray timeArray(dt, nPoints);

    // choose the state array to match the evolution of time
    using State = EuclideanState<2>;
    State start, end;
    start.setZero();
    end << 9, 9;
    DiscreteArray<State> stateArray(linspace<DiscreteArray<State>>(start, end, nPoints));

    // trajectory containers with zoh and linear interpolation
    DiscreteTrajectory<State> stateTrajectory_zoh(timeArray, stateArray, InterpolationType::ZOH);
    DiscreteTrajectory<State> stateTrajectory_lin(timeArray, stateArray, InterpolationType::LIN);


    for (size_t i = 0; i < timeArray.size(); i++)
    {
        ASSERT_EQ(stateArray[i], stateTrajectory_zoh.eval(i));
        ASSERT_EQ(stateArray[i], stateTrajectory_lin.eval(i));
        //		std::cout << "t: " << timeArray[i] << " x: " << stateArray[i].transpose() << " traj: " << stateTrajectory_lin.eval(i).transpose() << std::endl;
    }


    // check if copy-constructor works
    DiscreteTrajectory<State> stateTrajectory_copied(stateTrajectory_lin);
    for (double i = 0; i <= nPoints - 1; i = i + 0.2)
    {
        ASSERT_EQ(stateTrajectory_lin.eval(i), stateTrajectory_copied.eval(i));
    }


    // check if swapping works
    DiscreteTrajectory<State> stateTrajectory_swapped(InterpolationType::LIN);
    stateTrajectory_swapped.swapData(stateTrajectory_copied);
    for (double i = 0; i <= nPoints - 1; i = i + 0.2)
    {
        ASSERT_EQ(stateTrajectory_lin.eval(i), stateTrajectory_swapped.eval(i));
    }

    // check if accessors work
    ASSERT_EQ(start, stateTrajectory_lin.front());
    ASSERT_EQ(end, stateTrajectory_lin.back());
    ASSERT_EQ(end, stateTrajectory_lin[nPoints - 1]);


    // check get Time from index
    ASSERT_EQ(timeArray.back(), stateTrajectory_lin.getTimeFromIndex(nPoints - 1));


    // check time shifting
    StateTrajectory<2> stateTrajectory_shifted(stateTrajectory_lin);
    stateTrajectory_shifted.shiftTime(
        8.0);  // we shift the trajectory about 8 seconds, as a result the state vor time 0 should now be 8;
    ASSERT_EQ(stateArray[8], stateTrajectory_shifted.eval(0.0));                          // should be 8 8
    ASSERT_EQ(0.5 * (stateArray[8] + stateArray[9]), stateTrajectory_shifted.eval(0.5));  // should be 8.5 8.5


    // check if assignment works
    StateTrajectory<2> stateTrajectory_assigned = stateTrajectory_lin;
    for (double i = 0; i <= nPoints - 1; i = i + 0.2)
    {
        ASSERT_EQ(stateTrajectory_lin.eval(i), stateTrajectory_assigned.eval(i));
    }
}


TEST(TrajectoryTest, ManifoldTest)
{
    // a trajectory with 10 points, starting at t = 0 with dt = 1.0
    ct::core::Time dt = 1.0;
    size_t nPoints = 10;

    TimeArray timeArray(dt, nPoints);

    using State = ManifoldState<manif::SE3, manif::SE3Tangent>;
    State start, end;
    start.setIdentity();
    end.setRandom();
    DiscreteArray<State> stateArray(linspace<DiscreteArray<State>>(start, end, nPoints));

    // trajectory containers with zoh and linear interpolation
    DiscreteTrajectory<State> stateTrajectory_zoh(timeArray, stateArray, InterpolationType::ZOH);
    DiscreteTrajectory<State> stateTrajectory_lin(timeArray, stateArray, InterpolationType::LIN);


    for (size_t i = 0; i < timeArray.size(); i++)
    {
        ASSERT_TRUE(stateArray[i].isApprox(stateTrajectory_zoh.eval(i)));
        ASSERT_TRUE(stateArray[i].isApprox(stateTrajectory_lin.eval(i)));
        //		std::cout << "t: " << timeArray[i] << " x: " << stateArray[i].transpose() << " traj: " << stateTrajectory_lin.eval(i).transpose() << std::endl;
    }


    // check if copy-constructor works
    DiscreteTrajectory<State> stateTrajectory_copied(stateTrajectory_lin);
    for (double i = 0; i <= nPoints - 1; i = i + 0.2)
    {
        ASSERT_TRUE(stateTrajectory_lin.eval(i).isApprox(stateTrajectory_copied.eval(i)));
    }


    // check if swapping works
    DiscreteTrajectory<State> stateTrajectory_swapped(InterpolationType::LIN);
    stateTrajectory_swapped.swapData(stateTrajectory_copied);
    for (double i = 0; i <= nPoints - 1; i = i + 0.2)
    {
        ASSERT_TRUE(stateTrajectory_lin.eval(i).isApprox(stateTrajectory_swapped.eval(i)));
    }

    // check if accessors work
    ASSERT_TRUE(start.isApprox(stateTrajectory_lin.front()));
    ASSERT_TRUE(end.isApprox(stateTrajectory_lin.back()));
    ASSERT_TRUE(end.isApprox(stateTrajectory_lin[nPoints - 1]));


    // check get Time from index
    ASSERT_EQ(timeArray.back(), stateTrajectory_lin.getTimeFromIndex(nPoints - 1));
}


/*!
 *  \example DiscreteTrajectoryTest.cpp
 *
 *  This is a trivial test for the Discrete Trajectory class, which checks for basic functionality of its features.
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
