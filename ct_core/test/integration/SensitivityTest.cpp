/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include <gtest/gtest.h>

using namespace ct::core;

// test sensitivity computation for different types
void testEuclideanSensitivities(IntegrationType integrationType)
{
    // convenience typedefs
    const TIME_TYPE CT = CONTINUOUS_TIME;
    const size_t control_dim = 1;
    const size_t state_dim = 2;
    using State = EuclideanState<state_dim>;

    double dt = 0.01;
    size_t kSteps = 5;

    // system definition
    std::shared_ptr<ControlledSystem<State, control_dim, CT>> system(new SecondOrderSystem(10));
    std::shared_ptr<ConstantController<State, control_dim, CT>> controller(
        new ConstantController<State, control_dim, CT>);
    system->setController(controller);
    controller->setControl(ControlVector<1>::Random());

    // create a system linearizer
    std::shared_ptr<LinearSystem<State, control_dim, CT>> linearSystem(
        new SystemLinearizer<State, control_dim, CT>(system));


    // --- STEP 1 ---
    // create a substep recorder and hand it to an integrator
    std::shared_ptr<SubstepRecorder<State, control_dim>> substepRecorder(
        new SubstepRecorder<State, control_dim>(system));
    Integrator<State> integrator(system, integrationType, substepRecorder);

    typedef std::shared_ptr<DiscreteArray<State>> StateVectorArrayPtr;
    typedef std::shared_ptr<ControlVectorArray<control_dim>> ControlVectorArrayPtr;
    std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>> xSubstep(kSteps);
    std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>> uSubstep(kSteps);

    // create the sensitivity integrator
    SensitivityIntegrator<State, control_dim> sensIntegrator(dt, linearSystem, controller);
    sensIntegrator.setSubstepTrajectoryReference(&xSubstep, &uSubstep);


    // --- STEP 2 ---
    // create a classical sensitivity approximator (euler, tustin, etc)
    SensitivityApproximation<State, control_dim> sensApproximator(dt, linearSystem);

    // result variables
    StateMatrix<state_dim> A_approx, A_int;
    StateControlMatrix<state_dim, control_dim> B_approx, B_int;
    State state = State::Random();

    // integrate forward and compute respective sensitivities
    for (size_t i = 0; i < kSteps; i++)
    {
        State stateBefore = state;

        substepRecorder->reset();
        integrator.integrate_n_steps(state, 0.0, 1, dt);

        xSubstep[i] = substepRecorder->getSubstates();
        uSubstep[i] = substepRecorder->getSubcontrols();

        sensApproximator.getDerivatives(A_approx, B_approx, stateBefore, controller->getControl(), i);
        sensIntegrator.getDerivatives(A_int, B_int, stateBefore, controller->getControl(), i);

        // in case of Euler integration, the sensitivities should match exactly.
        // since we test with a linear system, the sensitivies should match exactly, when using RK4, too.
        ASSERT_TRUE(A_approx.isApprox(A_int));
        ASSERT_TRUE(B_approx.isApprox(B_int));
    }
}

TEST(SensitivityTest, compareEuler_euclidean)
{
    testEuclideanSensitivities(IntegrationType::EULERCT);
}

TEST(SensitivityTest, compareRK4_euclidean)
{
    testEuclideanSensitivities(IntegrationType::RK4CT);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
