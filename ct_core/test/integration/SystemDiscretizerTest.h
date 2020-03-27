/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <gtest/gtest.h>

using namespace ct::core;
using std::shared_ptr;


double uniformRandomNumber(double min, double max)
{
    std::random_device rd;                             // obtain a random number from hardware
    std::mt19937 eng(rd());                            // seed the generator
    std::uniform_real_distribution<> distr(min, max);  // define the range

    return distr(eng);
}


TEST(SystemDiscretizerTest, IntegratorComparison)
{
    const size_t state_dim = 2;
    const size_t control_dim = 1;
    using State = EuclideanState<state_dim>;

    size_t nTests = 10;

    for (size_t i = 1; i < nTests; ++i)
    {
        double w_n = uniformRandomNumber(0, 10);
        double zeta = uniformRandomNumber(0, 10);

        // make sure we are not complex
        while (w_n * w_n - zeta * zeta <= 0)
        {
            w_n = uniformRandomNumber(0, 100);
            zeta = uniformRandomNumber(0, 10);
        }

        // dynamic systems for manual integration and discrete propagation
        shared_ptr<SecondOrderSystem> oscillator1(new SecondOrderSystem(w_n, zeta));
        shared_ptr<SecondOrderSystem> oscillator2(new SecondOrderSystem(w_n, zeta));
        oscillator1->checkParameters();
        oscillator2->checkParameters();

        // set up a controller for the manually integrated system
        std::shared_ptr<ConstantController<State, control_dim, CONTINUOUS_TIME>> constantController(
            new ConstantController<State, control_dim, CONTINUOUS_TIME>());
        oscillator1->setController(constantController);

        // parameters for discretization
        double dt = 0.001;
        double startTime = 0.0;
        int nStages = 100;
        size_t K_sim = i;  // gradually increase K_sim according to the test id

        // integrators for comparison
        Integrator<State> integratorEuler(oscillator1, EULER);
        Integrator<State> integratorRK4(oscillator1, RK4);

        // system discretizer for comparison
        SystemDiscretizer<State, control_dim> systemDiscretizerEuler(oscillator2, dt, EULER, K_sim);
        SystemDiscretizer<State, control_dim> systemDiscretizerRK4(oscillator2, dt, RK4, K_sim);

        // initial states
        State integratedStateEuler, integratedStateRK4, propagatedStateEuler, propagatedStateRK4;
        State::Tangent euler_incr, rk4_incr;
        integratedStateEuler << 1.0, 0.0;
        integratedStateRK4 << 1.0, 0.0;
        propagatedStateEuler << 1.0, 0.0;
        propagatedStateRK4 << 1.0, 0.0;

        for (int j = 0; j < nStages; j++)
        {
            ct::core::ControlVector<control_dim> ctrl;
            ctrl(0) = uniformRandomNumber(-1.0, 1.0);

            // integrate systems
            constantController->setControl(ctrl);
            integratorEuler.integrate_n_steps(integratedStateEuler, startTime, K_sim, dt / (double)K_sim);
            integratorRK4.integrate_n_steps(integratedStateRK4, startTime, K_sim, dt / (double)K_sim);

            // propagate system and summarize increments
            systemDiscretizerEuler.computeControlledDynamics(propagatedStateEuler, j, ctrl, euler_incr);
            systemDiscretizerRK4.computeControlledDynamics(propagatedStateRK4, j, ctrl, rk4_incr);
            propagatedStateEuler = propagatedStateEuler + euler_incr;
            propagatedStateRK4 = propagatedStateRK4 + rk4_incr;
            
            // the manually integrated as well as the propagated state need to be the same.
            ASSERT_LT((propagatedStateEuler - integratedStateEuler).array().abs().maxCoeff(), 1e-6);
            ASSERT_LT((propagatedStateRK4 - integratedStateRK4).array().abs().maxCoeff(), 1e-6);
        }
    }
}
