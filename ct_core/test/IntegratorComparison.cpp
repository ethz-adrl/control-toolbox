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


#include <gtest/gtest.h>
#include <ct/core/core.h>
#include <ct/core/integration/IntegratorCT.h>
#include "system/TestNonlinearSystem.h"


using namespace ct::core;
using std::shared_ptr;


double randomNumber(double min, double max)
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_real_distribution<> distr(min, max); // define the range

    return distr(eng);
}


TEST(IntegrationTest, derivativeTest)
{
    const size_t state_dim = TestNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable system
    typedef CppAD::AD<double> AD_Scalar;
    typedef tpl::TestNonlinearSystem<AD_Scalar> TestNonlinearSystemAD;

    // handy typedefs for the Jacobian
    typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
    typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    // 
    size_t nTests = 1;

    for(size_t i = 0; i < nTests; ++i)
    {
        shared_ptr<SecondOrderSystem > oscillator;
        double w_n = randomNumber(0, 10);
        double zeta = randomNumber(0, 10);

        // make sure we are not complex
        while (w_n*w_n - zeta*zeta <= 0)
        {
            w_n = randomNumber(0, 100);
            zeta = randomNumber(0, 10);
        }
        oscillator = shared_ptr<SecondOrderSystem >(new SecondOrderSystem(w_n, zeta));
        oscillator->checkParameters();
        // shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<AD_Scalar>(AD_Scalar(w_n)));

        std::shared_ptr<IntegratorBase<state_dim> > integratorOdeint;
        integratorOdeint = std::shared_ptr<IntegratorEuler<state_dim> >(new IntegratorEuler<state_dim>(oscillator));

        IntegratorEulerCT<state_dim> integratorCT(oscillator);

        StateVector<state_dim> stateCT; stateCT << 1.0, 0.0;
        StateVector<state_dim> stateOdeInt; stateOdeInt << 1.0, 0.0;
        double dt = 0.00001;
        double startTime = 0.0;
        size_t numSteps = 100000;

        Timer t;

        t.start();

        integratorOdeint->integrate_n_steps(stateOdeInt, startTime, numSteps, dt);

        t.stop();

        std::cout << "ODEINT took: " << t.getElapsedTime() << " s for integration" << std::endl;

        t.reset();
        t.start();
        integratorCT.integrate(stateCT, startTime, numSteps, dt);

        t.stop();
        std::cout << "CT took: " << t.getElapsedTime() << " s for integration" << std::endl;


        ASSERT_LT((stateCT-stateOdeInt).array().abs().maxCoeff(), 1e-6);
    }

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}