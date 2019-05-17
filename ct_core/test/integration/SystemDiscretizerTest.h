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
        std::shared_ptr<ConstantController<state_dim, control_dim>> constantController(
            new ConstantController<state_dim, control_dim>());
        oscillator1->setController(constantController);

        // parameters for discretization
        double dt = 0.001;
        double startTime = 0.0;
        size_t nStages = 100;
        size_t K_sim = i;  // gradually increase K_sim according to the test id

        // integrators for comparison
        Integrator<state_dim> integratorEuler(oscillator1, EULER);
        Integrator<state_dim> integratorRK4(oscillator1, RK4);

        // system discretizer for comparison
        SystemDiscretizer<state_dim, control_dim> systemDiscretizerEuler(oscillator2, dt, EULER, K_sim);
        SystemDiscretizer<state_dim, control_dim> systemDiscretizerRK4(oscillator2, dt, RK4, K_sim);

        // initial states
        StateVector<state_dim> integratedStateEuler, integratedStateRK4, propagatedStateEuler, propagatedStateRK4;
        integratedStateEuler << 1.0, 0.0;
        integratedStateRK4 << 1.0, 0.0;
        propagatedStateEuler << 1.0, 0.0;
        propagatedStateRK4 << 1.0, 0.0;

        for (size_t j = 0; j < nStages; j++)
        {
            ct::core::ControlVector<control_dim> ctrl;
            ctrl(0) = uniformRandomNumber(-1.0, 1.0);

            // integrate systems
            constantController->setControl(ctrl);
            integratorEuler.integrate_n_steps(integratedStateEuler, startTime, K_sim, dt / (double)K_sim);
            integratorRK4.integrate_n_steps(integratedStateRK4, startTime, K_sim, dt / (double)K_sim);

            // propagate systems
            systemDiscretizerEuler.propagateControlledDynamics(propagatedStateEuler, j, ctrl, propagatedStateEuler);
            systemDiscretizerRK4.propagateControlledDynamics(propagatedStateRK4, j, ctrl, propagatedStateRK4);

            // the manually integrated as well as the propagated state need to be the same.
            ASSERT_LT((propagatedStateEuler - integratedStateEuler).array().abs().maxCoeff(), 1e-12);
            ASSERT_LT((propagatedStateRK4 - integratedStateRK4).array().abs().maxCoeff(), 1e-12);
        }
    }
}
