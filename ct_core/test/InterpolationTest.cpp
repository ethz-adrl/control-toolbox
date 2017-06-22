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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

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


	for(int i=1; i< 5; i++)
	{
		double enquiryTime = 0.5*i;

		StateVector<2> enquiryData;
		linInterpolation.interpolate(timeStamp, data, enquiryTime, enquiryData);

		std::cout << "At time " << enquiryTime << "\t data is " <<  enquiryData.transpose() << std::endl;

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
