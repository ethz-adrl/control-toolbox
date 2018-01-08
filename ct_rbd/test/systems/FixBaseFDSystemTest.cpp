/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <iostream>

#include <gtest/gtest.h>

#include "../models/testIrb4600/RobCoGenTestIrb4600.h"

using namespace ct;
using namespace ct::rbd;

TEST(FixBaseFDSystemTest, ForwardDynamicsTest)
{
    const size_t STATE_DIM = FixBaseFDSystem<TestIrb4600::Dynamics>::STATE_DIM;

    std::shared_ptr<core::System<STATE_DIM>> dynamics(new FixBaseFDSystem<TestIrb4600::Dynamics>);

    core::Integrator<STATE_DIM> integrator(dynamics, ct::core::RK4);

    core::StateVector<12> state;
    state.setZero();

    std::cout << "___________________________________________________________________________" << std::endl;

    std::cout << "Init state: " << state.transpose() << std::endl;

    integrator.integrate_n_steps(state, 0, 1000, 0.001);

    std::cout << "Integrated state: " << state.transpose() << std::endl;

    std::cout << "___________________________________________________________________________" << std::endl;
}


TEST(FixBaseFDSystemTest, ActuatorDynamicsTest)
{
    const size_t rbd_state_dim = FixBaseFDSystem<TestIrb4600::Dynamics>::STATE_DIM;
    const size_t njoints = TestIrb4600::Dynamics::NJOINTS;
    const size_t actuator_state_dim = 2 * njoints;

    const size_t state_dim = rbd_state_dim + actuator_state_dim;

    // generate actuator dynamics
    const double w_n = 2;
    const double zeta = 1;  // critical damping
    const double gc =
        w_n * w_n;  // select oscillator input gain such that we follow the reference input with no amplification
    std::shared_ptr<ct::rbd::SecondOrderActuatorDynamics<njoints>> actDynamics(
        new ct::rbd::SecondOrderActuatorDynamics<njoints>(w_n, zeta, gc));

    // generate fix-base dynamics
    std::shared_ptr<FixBaseFDSystem<TestIrb4600::Dynamics, actuator_state_dim>> combinedDynamics(
        new FixBaseFDSystem<TestIrb4600::Dynamics, actuator_state_dim>(actDynamics));

    // generate random control action and apply to the system
    ct::core::ControlVector<njoints> constantControl;
    constantControl.setRandom();
    std::cout << "constant control " << constantControl.transpose() << std::endl;
    std::shared_ptr<ct::core::ConstantController<state_dim, njoints>> constantController(
        new ct::core::ConstantController<state_dim, njoints>(constantControl));
    combinedDynamics->setController(constantController);

    // make integrator and integrate the combined system
    core::Integrator<state_dim> integrator(combinedDynamics, ct::core::RK4);

    core::StateVector<state_dim> state;
    state.setZero();

    std::cout << "___________________________________________________________________________" << std::endl;

    std::cout << "Init state: " << state.transpose() << std::endl;

    size_t nSteps = 5000;
    double dt_sim = 0.005;
    integrator.integrate_n_steps(state, 0, nSteps, dt_sim);

    std::cout << "Integrated overall state: " << state.transpose() << std::endl;

    std::cout << "___________________________________________________________________________" << std::endl;


    TestIrb4600::Dynamics::RBDState_t rbdState = combinedDynamics->RBDStateFromVector(state);
    std::cout << "Integrated joint positions: " << rbdState.joints().getPositions().transpose() << std::endl;
    std::cout << "Integrated joint velocities: " << rbdState.joints().getVelocities().transpose() << std::endl;

    ct::core::StateVector<actuator_state_dim> actState = combinedDynamics->actuatorStateFromVector(state);
    std::cout << "Integrated actuator state: " << actState.transpose() << std::endl;
    std::cout << "___________________________________________________________________________" << std::endl;

    // in the limit, the actuator states must be identical to the constant control
    for (size_t i = 0; i < njoints; i++)
        ASSERT_NEAR(actState(i), constantControl(i), 1e-4);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
