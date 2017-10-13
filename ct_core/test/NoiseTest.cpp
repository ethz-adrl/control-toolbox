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

#include <cmath>
#include <memory>
#include <random>

#include <ct/core/core.h>

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;

double randomNumber(double min, double max)
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_real_distribution<> distr(min, max); // define the range

    return distr(eng);
}

TEST(NoiseTest, gaussianNoiseTest)
{
    size_t nTests = 100;
    size_t nSamples = 100000;

    for (size_t i=0; i<nTests; i++)
    {
        double mean = randomNumber(-999, 999);
        double stdDev = randomNumber(0, 10);
        GaussianNoise gNoise(mean, stdDev);

        double meanMeas = 0;
        double variance = 0;

        // draw samples
        for (size_t j=0; j<nSamples; j++)
        {
            double sample = gNoise();
            meanMeas += sample/nSamples;

            double deviation = (sample - mean);
            variance += (deviation*deviation) / nSamples;
        }

        // allow 5% error
        ASSERT_NEAR(mean, meanMeas, std::fabs(stdDev/20.0));

        double stdDevMeasured = std::sqrt(variance);
        ASSERT_NEAR(stdDev, stdDevMeasured, stdDev/15.0);
    }
}



TEST(NoiseTest, quantizationNoiseTest)
{
    size_t nTests = 1000;
    size_t nSamples = 10000;

    double bias = 0;

    for (size_t i=0; i<nTests; i++)
    {
        double mean = randomNumber(-999, 999);
        double quantizationInterval = randomNumber(0, 1);
        QuantizationNoise qNoise(mean, quantizationInterval);

        // gaussian noise as measurement
        GaussianNoise gNoise(0.0, 10*quantizationInterval);

        double meanMeas = 0;
        double lastSample = 0;

        // draw samples
        for (size_t j=0; j<nSamples; j++)
        {
            double sample = gNoise();
            qNoise.noisify(sample);

            if (j>0)
            {
                double quantization = std::fmod(std::fabs(sample-lastSample) + quantizationInterval/1000.0, quantizationInterval);
                ASSERT_NEAR(std::fabs(quantization), 0.0, quantizationInterval/100.0);
            }

            meanMeas += sample/nSamples;

            lastSample = sample;
        }

        ASSERT_NEAR(mean, meanMeas, 999.0/1000.0);

        // average percentage of mean that forms the bias
        bias += ((meanMeas-mean)/mean)/nTests;
    }
    // allow 1% error
    ASSERT_NEAR(bias, 0, 1e-2);
}



/*!
 *  \example NoiseTest.cpp
 *
 *  This is a trivial test for the Gaussian Noise class but also serves as implementation example
 */
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
