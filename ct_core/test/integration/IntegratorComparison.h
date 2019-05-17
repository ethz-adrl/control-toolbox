/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <gtest/gtest.h>
#include "../system/TestNonlinearSystem.h"


using namespace ct::core;
using std::shared_ptr;


double uniformRandomNumber(double min, double max)
{
    std::random_device rd;                             // obtain a random number from hardware
    std::mt19937 eng(rd());                            // seed the generator
    std::uniform_real_distribution<> distr(min, max);  // define the range

    return distr(eng);
}


TEST(IntegrationTest, derivativeTest)
{
    const size_t state_dim = TestNonlinearSystem::STATE_DIM;

    // create two nonlinear systems, one regular one and one auto-differentiable
    //
    size_t nTests = 1;

    for (size_t i = 0; i < nTests; ++i)
    {
        shared_ptr<SecondOrderSystem> oscillator;
        double w_n = uniformRandomNumber(0, 10);
        double zeta = uniformRandomNumber(0, 10);

        // make sure we are not complex
        while (w_n * w_n - zeta * zeta <= 0)
        {
            w_n = uniformRandomNumber(0, 100);
            zeta = uniformRandomNumber(0, 10);
        }
        oscillator = shared_ptr<SecondOrderSystem>(new SecondOrderSystem(w_n, zeta));
        oscillator->checkParameters();

        // std::shared_ptr<IntegratorBase<state_dim> > integratorEulerOdeint;
        Integrator<state_dim> integratorEulerOdeint(oscillator, EULER);
        // std::shared_ptr<IntegratorBase<state_dim> > integratorRk4Odeint;
        Integrator<state_dim> integratorRk4Odeint(oscillator, RK4);

        Integrator<state_dim> integratorEulerCT(oscillator, EULERCT);
        Integrator<state_dim> integratorRK4CT(oscillator, RK4CT);

        StateVector<state_dim> stateEulerCT;
        stateEulerCT << 1.0, 0.0;
        StateVector<state_dim> stateEulerOdeInt;
        stateEulerOdeInt << 1.0, 0.0;
        StateVector<state_dim> stateRK4CT;
        stateRK4CT << 1.0, 0.0;
        StateVector<state_dim> stateRK4OdeInt;
        stateRK4OdeInt << 1.0, 0.0;

        double dt = 0.00001;
        double startTime = 0.0;
        size_t numSteps = 100000;

        Timer t;
        t.start();
        integratorEulerOdeint.integrate_n_steps(stateEulerOdeInt, startTime, numSteps, dt);
        t.stop();

        std::cout << "ODEINT Euler took: " << t.getElapsedTime() << " s for integration" << std::endl;

        t.reset();
        t.start();
        integratorEulerCT.integrate_n_steps(stateEulerCT, startTime, numSteps, dt);
        t.stop();
        std::cout << "CT Euler took: " << t.getElapsedTime() << " s for integration" << std::endl;

        t.reset();
        t.start();
        integratorRk4Odeint.integrate_n_steps(stateRK4OdeInt, startTime, numSteps, dt);
        t.stop();
        std::cout << "ODEINT RK4 took: " << t.getElapsedTime() << " s for integration" << std::endl;

        t.reset();
        t.start();
        integratorRK4CT.integrate_n_steps(stateRK4CT, startTime, numSteps, dt);
        t.stop();
        std::cout << "CT RK4 took: " << t.getElapsedTime() << " s for integration" << std::endl;

        ASSERT_LT((stateRK4CT - stateRK4OdeInt).array().abs().maxCoeff(), 1e-12);
        ASSERT_LT((stateEulerCT - stateEulerOdeInt).array().abs().maxCoeff(), 1e-12);
    }
}
