/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include <iostream>
#include <cstdlib>

#include <ct/core/core.h>

#include <gtest/gtest.h>

using namespace ct::core;


/**
 * this implements a trivial test for the trajectory class
 */
TEST(TrajectoryTest, TrajectoryTest)
{
    // a trajectory with 10 points, starting at t = 0 with dt = 1.0
    ct::core::Time dt = 1.0;
    size_t nPoints = 10;

    TimeArray timeArray(dt, nPoints);

    // choose the state array to match the evolution of time
    StateVector<2> start;
    start.setZero();
    StateVector<2> end;
    end << 9, 9;
    StateVectorArray<2> stateArray(linspace<StateVectorArray<2>>(start, end, nPoints));

    // trajectory containers with zoh and linear interpolation
    StateTrajectory<2> stateTrajectory_zoh(timeArray, stateArray, InterpolationType::ZOH);
    StateTrajectory<2> stateTrajectory_lin(timeArray, stateArray, InterpolationType::LIN);


    for (size_t i = 0; i < timeArray.size(); i++)
    {
        ASSERT_EQ(stateArray[i], stateTrajectory_lin.eval(i));
        //		std::cout << "t: " << timeArray[i] << " x: " << stateArray[i].transpose() << " traj: " << stateTrajectory_lin.eval(i).transpose() << std::endl;
    }


    // check if copy-constructor works
    StateTrajectory<2> stateTrajectory_copied(stateTrajectory_lin);
    for (double i = 0; i <= nPoints - 1; i = i + 0.2)
    {
        ASSERT_EQ(stateTrajectory_lin.eval(i), stateTrajectory_copied.eval(i));
    }


    // check if swapping works
    StateTrajectory<2> stateTrajectory_swapped(InterpolationType::LIN);
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
