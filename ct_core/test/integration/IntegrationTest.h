/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <memory>


using namespace ct::core;
using std::shared_ptr;

const size_t stateSize = 2;
bool plotResult = false;

std::vector<std::string> integratorNames = {"euler", "rk4", "rk78", "rk5 variable", "ode45", "modified midpoint"};
std::vector<std::string> integratorCalls = {"const", "nSteps", "adaptive", "times"};

double uniformRandomNumber(double min, double max)
{
    std::random_device rd;                             // obtain a random number from hardware
    std::mt19937 eng(rd());                            // seed the generator
    std::uniform_real_distribution<> distr(min, max);  // define the range

    return distr(eng);
}

void plotResults(std::vector<StateVectorArray<stateSize>>& stateTrajectories, std::vector<TimeArray>& timeTrajectories)
{
#ifdef PLOTTING_ENABLED

    plot::ion();
    plot::figure();

    std::cout << "plotting " << stateTrajectories.size() << " trajectories" << std::endl;
    for (size_t i = 0; i < stateTrajectories.size(); i++)
    {
        if (timeTrajectories[i].size() != stateTrajectories[i].size())
            throw std::runtime_error("Cannot plot data, x and y not equal length");

        std::vector<double> position;
        for (size_t j = 0; j < stateTrajectories[i].size(); j++)
        {
            position.push_back(stateTrajectories[i][j](0) + (stateTrajectories.size() - i - 1));
        }

        int type = ((int)i) / 4;
        int call = i % 4;
        std::string name = integratorNames[type] + " - " + integratorCalls[call];

        plot::subplot(5, std::round(stateTrajectories.size() / 5.0) + 1, i + 1);
        plot::plot(timeTrajectories[i].toImplementation(), position);
        plot::title(name);
        //        std::cout << "time: "<< std::endl;
        //        for (auto k: timeTrajectories[i])
        //          std::cout << k << ' ';
        //
        //        std::cout << "pos: "<< std::endl;
        //        for (auto k: position)
        //		  std::cout << k << ' ';
    }

    plot::show(false);
#endif
}

TEST(IntegrationTest, derivativeTest)
{
    try
    {
        // will be initialized later
        shared_ptr<SecondOrderSystem> oscillator;

        // create a 2 state integrator
        std::vector<std::shared_ptr<Integrator<stateSize>>> integrators;

        double dt = 0.0001;
        size_t nTests = 5;

        for (size_t i = 0; i < nTests; i++)
        {
            // define integration times
            Time startTime = 0.0;
            Time finalTime = startTime + 1.0;
            size_t nsteps = 1.0 / dt;

            // create an initial state
            StateVector<stateSize> initialState;
            initialState << 1.0, 0.0;

            // create a 10 Hz second order system with damping 0.1
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
            //oscillator->printSystemInfo();

            integrators.clear();
            integrators.push_back(std::shared_ptr<Integrator<stateSize>>(new Integrator<stateSize>(oscillator, EULER)));
            integrators.push_back(std::shared_ptr<Integrator<stateSize>>(new Integrator<stateSize>(oscillator, RK4)));
            integrators.push_back(std::shared_ptr<Integrator<stateSize>>(new Integrator<stateSize>(oscillator, RK78)));
            integrators.push_back(
                std::shared_ptr<Integrator<stateSize>>(new Integrator<stateSize>(oscillator, RK5VARIABLE)));
            integrators.push_back(std::shared_ptr<Integrator<stateSize>>(new Integrator<stateSize>(oscillator, ODE45)));
            integrators.push_back(
                std::shared_ptr<Integrator<stateSize>>(new Integrator<stateSize>(oscillator, MODIFIED_MIDPOINT)));


            // define analytical solution
            // we start at 1 so we have 1/2 phase
            double w_d = std::sqrt(w_n * w_n - zeta * zeta);
            double phase = std::atan(w_d / zeta);
            double A = 1.0 / std::sin(phase);
            auto solution = [w_d, zeta, phase, A](
                Time t) { return A * std::exp(-zeta * t) * std::sin(w_d * t + phase); };


            // create trajectory containers
            size_t integratorFunctions = 4;  // integrate_const, integrate_n_steps, ...
            size_t nIntegrators = integrators.size();
            size_t nResults = nIntegrators * integratorFunctions;

            std::vector<StateVectorArray<stateSize>> stateTrajectories(nResults);
            std::vector<TimeArray> timeTrajectories(nResults);
            std::vector<StateVector<stateSize>> finalStates(nResults);

            StateVector<stateSize> initialStateLocal;

            for (size_t j = 0; j < nIntegrators; j++)
            {
                //std::cout << "Testing integrator: " << integratorNames[j] << std::endl;

                initialStateLocal = initialState;

                //std::cout << "Testing integration call const" << std::endl;

                // use fixed step integration
                integrators[j]->integrate_const(initialStateLocal, startTime, finalTime, dt,
                    stateTrajectories[j * integratorFunctions + 0], timeTrajectories[j * integratorFunctions + 0]);

                initialStateLocal = initialState;

                //std::cout << "Testing integration call integrate_n_steps" << std::endl;

                integrators[j]->integrate_n_steps(initialStateLocal, startTime, nsteps, dt,
                    stateTrajectories[j * integratorFunctions + 1], timeTrajectories[j * integratorFunctions + 1]);

                initialStateLocal = initialState;

                //std::cout << "Testing integration call integrate adaptive" << std::endl;

                integrators[j]->integrate_adaptive(initialStateLocal, startTime, finalTime,
                    stateTrajectories[j * integratorFunctions + 2], timeTrajectories[j * integratorFunctions + 2], dt);

                initialStateLocal = initialState;

                for (size_t k = 0; k < nsteps + 1; k++)
                {
                    timeTrajectories[j * integratorFunctions + 3].push_back(k * dt);
                }

                //std::cout << "Testing integration call integrate times" << std::endl;

                integrators[j]->integrate_times(initialStateLocal, timeTrajectories[j * integratorFunctions + 3],
                    stateTrajectories[j * integratorFunctions + 3], dt);
            }

            for (size_t j = 0; j < nResults; j++)
            {
                //std::cout << "Testing result number " << j << std::endl;

                // we should get at least two points, start and end
                ASSERT_GT(stateTrajectories[j].size(), 2);
                ASSERT_GT(timeTrajectories[j].size(), 2);

                // we should get equal number of states and times
                ASSERT_LT((stateTrajectories[j].front() - initialState).array().abs().maxCoeff(), 1e-6);
                ASSERT_EQ(stateTrajectories[j].size(), timeTrajectories[j].size());

                // start and end should be correct
                ASSERT_NEAR(timeTrajectories[j].front(), startTime, dt);
                ASSERT_NEAR(timeTrajectories[j].back(), finalTime, dt);

                // check ordering of time stamps
                for (size_t k = 1; k < timeTrajectories[j].size(); k++)
                {
                    ASSERT_GT(timeTrajectories[j][k], timeTrajectories[j][k - 1]);

                    if (j % 4 == 0 || j % 4 == 1)
                    {
                        // check equidistance
                        ASSERT_NEAR(timeTrajectories[j][k] - timeTrajectories[j][k - 1], dt, 1e-6);
                    }
                }

                // check correctness, only valid for non-adaptive
                if (j % 4 - 2 != 0)
                {
                    for (size_t k = 1; k < stateTrajectories[j].size(); k++)
                    {
                        double derivativeNumDiff = stateTrajectories[j][k](0) - stateTrajectories[j][k - 1](0);
                        double derivativeAnalytical =
                            solution(timeTrajectories[j][k]) - solution(timeTrajectories[j][k - 1]);

                        ASSERT_NEAR(derivativeNumDiff, derivativeAnalytical, 1e-2);
                    }
                }

                if (j > 1)
                {
                    ASSERT_NEAR(stateTrajectories[j].back()(0), stateTrajectories[j - 1].back()(0), 5e-2);
                }
            }

            if (plotResult)
                plotResults(stateTrajectories, timeTrajectories);
        }
#ifdef PLOTTING_ENABLED
        if (plotResult)
            plot::show(true);
#endif
    } catch (...)
    {
        std::cout << "Caught exception." << std::endl;
        FAIL();
    }
}
