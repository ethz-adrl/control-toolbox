/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <iostream>
#include <cstdlib>

#include <ct/core/core.h>
#include <gtest/gtest.h>

using namespace ct::core;

TEST(InterplationTest, Linear)
{
    TimeArray timeStamp(5);
    timeStamp[0] = 0.0;
    timeStamp[1] = 0.5;
    timeStamp[2] = 1.0;
    timeStamp[3] = 1.5;
    timeStamp[4] = 2.0;

    StateVectorArray<2> data(5);
    data[0] << 0.0, 0.0;
    data[1] << 1.0, 0.0;
    data[2] << 2.0, 0.0;
    data[3] << 3.0, 0.0;
    data[4] << 4.0, 0.0;

    ct::core::Interpolation<StateVector<2>> linInterpolation(InterpolationType::LIN);


    for (int i = 1; i < 5; i++)
    {
        double enquiryTime = 0.5 * i;

        StateVector<2> enquiryData;
        linInterpolation.interpolate(timeStamp, data, enquiryTime, enquiryData);

        std::cout << "At time " << enquiryTime << "\t data is " << enquiryData.transpose() << std::endl;

        Eigen::Vector2d nominal_result;
        nominal_result << double(i), 0.0;

        ASSERT_LT((nominal_result - enquiryData).array().abs().maxCoeff(), 1e-6);
    }
}


/*!
 *  \example InterpolationTest.cpp
 *
 *  This unit test tests the interpolation class but also serves as minimal example how to perform interpolation given a data and time array
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
