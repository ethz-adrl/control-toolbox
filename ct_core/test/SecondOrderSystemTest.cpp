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


#include <cmath>
#include <memory>

#include <ct/core/core.h>

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;
using std::shared_ptr;

double randomNumber(double min, double max)
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_real_distribution<> distr(min, max); // define the range

    return distr(eng);
}

TEST(SecondOrderSystemTest, dynamicsTest)
{
    try {

        const size_t stateSize = 2;

        // will be initialized later
        shared_ptr<SecondOrderSystem > oscillator;

        size_t nTests = 100;

        for (size_t i=0; i<nTests; i++)
        {
            // create a 10 Hz second order system with damping 0.1
            double w_n;
            double zeta;

            w_n = randomNumber(0, 1000);
            zeta = 0.0;

            oscillator = shared_ptr<SecondOrderSystem >(new SecondOrderSystem(w_n, zeta));
            oscillator->checkParameters();

            // define analytical solution
            // we start at 1 so we have 1/2 phase
            double w_d = std::sqrt(w_n*w_n - zeta*zeta);
            double phase = 0;
            double A = 1.0;
            auto solution = [w_d, zeta, phase, A](Time t) { return A*std::sin(w_d*t + phase); };
            auto der = [w_d, zeta, phase, A](Time t) { return A*std::cos(w_d*t + phase)*w_d; };

            size_t nSamples = 100;

            // create a state
            StateVector<stateSize> state;
            ControlVector<1> control;
            control.setZero();

            for (size_t j=0; j<nSamples; j++)
            {
                StateVector<2> derivative;
                double t = randomNumber(0, 1000);

                state(0) = solution(t);
                state(1) = der(t);

                oscillator->computeControlledDynamics(state, t, control, derivative);

                double derivativeNumDiff;
                double dtNumDiff = 1e-6;
                derivativeNumDiff = (solution(t+dtNumDiff) - solution(t-dtNumDiff))/(2*dtNumDiff);

                double derivativeAnalytic = der(t);

                ASSERT_NEAR(derivative(0), derivativeNumDiff, 1e-3);
                ASSERT_NEAR(derivativeAnalytic, derivativeNumDiff, 1e-3);
            }
        }
    } catch (...)
    {
        std::cout << "Caught exception." << std::endl;
        FAIL();
    }
}


/*!
 *  \example SecondOrderSystemTest.cpp
 *
 *  This is a trivial test for the Oscillator.
 */
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
