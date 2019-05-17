/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>
#include <ct/core/core.h>
#include "../system/TestSymplecticSystem.h"


using namespace ct::core;
using std::shared_ptr;

const size_t state_dim = 2;
const size_t p_dim = 1;
const size_t v_dim = 1;
const size_t control_dim = 1;

double uniformRandomNumber(double min, double max)
{
    std::random_device rd;                             // obtain a random number from hardware
    std::mt19937 eng(rd());                            // seed the generator
    std::uniform_real_distribution<> distr(min, max);  // define the range

    return distr(eng);
}

TEST(SymplecticIntegrationTest, simpleSymplecticTest)
{
    try
    {
        double dt = 0.0001;
        size_t nTests = 5;

        for (size_t i = 0; i < nTests; i++)
        {
            double wn = uniformRandomNumber(0.1, 1.0);
            shared_ptr<ConstantController<state_dim, control_dim>> constController(
                new ConstantController<state_dim, control_dim>());
            shared_ptr<TestSymplecticSystem> oscillator(new TestSymplecticSystem(wn, constController));

            ct::core::ControlVector<control_dim> testControl;
            testControl(0) = uniformRandomNumber(-1, 1);
            constController->setControl(testControl);

            // create symplectic integrators
            std::shared_ptr<ct::core::IntegratorSymplecticEuler<p_dim, v_dim, control_dim>> symplecticEuler(
                new ct::core::IntegratorSymplecticEuler<p_dim, v_dim, control_dim>(oscillator));
            std::shared_ptr<ct::core::IntegratorSymplecticRk<p_dim, v_dim, control_dim>> symplecticRK4(
                new ct::core::IntegratorSymplecticRk<p_dim, v_dim, control_dim>(oscillator));

            // define integration times
            Time startTime = 0.0;
            size_t nsteps = 1.0 / dt;

            // create an initial state
            StateVector<state_dim> state1, state2;
            state1 << 1.0, 0.0;
            state2 << 1.0, 0.0;

            // integrate forward with symplectic integrators
            symplecticEuler->integrate_n_steps(state1, startTime, nsteps, dt);
            symplecticRK4->integrate_n_steps(state2, startTime, nsteps, dt);

            // ASSERT that the two states integrated by symplectic integrators are close
            ASSERT_NEAR(state1.array().abs().maxCoeff(), state2.array().abs().maxCoeff(), 1e-3);
        }
    } catch (...)
    {
        std::cout << "Caught exception." << std::endl;
        FAIL();
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
