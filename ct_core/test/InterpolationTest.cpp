/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <iostream>
#include <cstdlib>

#include <ct/core/core.h>
#include <gtest/gtest.h>

using namespace ct::core;

const bool verbose = false;

TEST(InterplationTest, LinearEuclidean)
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

        if (verbose)
            std::cout << "At time " << enquiryTime << "\t data is " << enquiryData.transpose() << std::endl;

        Eigen::Vector2d nominal_result;
        nominal_result << double(i), 0.0;

        ASSERT_LT((nominal_result - enquiryData).array().abs().maxCoeff(), 1e-6);
    }
}


TEST(InterplationTest, LinearManifold)
{
    TimeArray timeStamp(5);
    timeStamp[0] = 0.0;
    timeStamp[1] = 0.5;
    timeStamp[2] = 1.0;
    timeStamp[3] = 1.5;
    timeStamp[4] = 2.0;

    using State = ct::core::ManifoldState<manif::SE2, manif::SE2Tangent, double>;
    DiscreteArray<State> data(5);
    data[0] = manif::SE2d(0.0, 0.0, std::cos(0.0), std::sin(0.0));
    data[1] = manif::SE2d(1.0, 0.0, std::cos(M_PI / 4), std::sin(M_PI / 4));
    data[2] = manif::SE2d(2.0, 0.0, std::cos(M_PI / 2), std::sin(M_PI / 2));
    data[3] = manif::SE2d(3.0, 0.0, std::cos(3 * M_PI / 4), std::sin(3 * M_PI / 4));
    data[4] = manif::SE2d(4.0, 0.0, std::cos(M_PI), std::sin(M_PI));

    ct::core::Interpolation<State> linInterpolation(InterpolationType::LIN);


    for (int i = 1; i < 5; i++)
    {
        double enquiryTime = 0.5 * i;

        State enquiryData;
        linInterpolation.interpolate(timeStamp, data, enquiryTime, enquiryData);

        if (verbose)
            std::cout << "At time " << enquiryTime << "\t data is " << enquiryData << std::endl;

        Eigen::Vector4d nominal_result;
        nominal_result << double(i), 0.0, std::cos(i * M_PI / 4), std::sin(i * M_PI / 4);

        ASSERT_TRUE(nominal_result.isApprox(enquiryData.coeffs()));
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
