/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>

using namespace ct::core;

void testSensitivities(IntegrationType integrationType)
{
    std::shared_ptr<ControlledSystem<2, 1>> system(new SecondOrderSystem(10));
    std::shared_ptr<ConstantController<2, 1>> controller(new ConstantController<2, 1>);
    system->setController(controller);
    controller->setControl(ControlVector<1>::Random());

    std::shared_ptr<SubstepRecorder<2, 1>> substepRecorder(new SubstepRecorder<2, 1>(system));
    Integrator<2> integrator(system, integrationType, substepRecorder);

    std::shared_ptr<SystemLinearizer<2, 1>> linearSystem(new SystemLinearizer<2, 1>(system));

    typedef std::shared_ptr<Sensitivity<2, 1>> SensitivityPtr;
    std::vector<SensitivityPtr, Eigen::aligned_allocator<SensitivityPtr>> sensitivities;

    double dt = 0.01;

    sensitivities.push_back(SensitivityPtr(new SensitivityApproximation<2, 1>(dt, linearSystem)));
    sensitivities.push_back(SensitivityPtr(new SensitivityIntegrator<2, 1>(dt, linearSystem, controller)));


    size_t kSteps = 5;

    StateVector<2> state;
    state.setRandom();

    typedef std::shared_ptr<StateVectorArray<2>> StateVectorArrayPtr;
    typedef std::shared_ptr<ControlVectorArray<1>> ControlVectorArrayPtr;
    std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>> xSubstep(kSteps);
    std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>> uSubstep(kSteps);

    for (size_t j = 0; j < sensitivities.size(); j++)
    {
        sensitivities[j]->setSubstepTrajectoryReference(&xSubstep, &uSubstep);
    }

    for (size_t i = 0; i < kSteps; i++)
    {
        StateVector<2> stateBefore = state;

        substepRecorder->reset();
        integrator.integrate_n_steps(state, 0.0, 1, dt);

        for (size_t j = 0; j < sensitivities.size(); j++)
        {
            xSubstep[i] = substepRecorder->getSubstates();
            uSubstep[i] = substepRecorder->getSubcontrols();

            StateMatrix<2> A;
            StateControlMatrix<2, 1> B;

            sensitivities[j]->getAandB(stateBefore, controller->getControl(), state, i, 1, A, B);

            std::cout << "A:" << std::endl << A << std::endl << std::endl;
            std::cout << "B:" << std::endl << B << std::endl << std::endl;

            std::cout << std::endl << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    testSensitivities(IntegrationType::EULERCT);
    testSensitivities(IntegrationType::RK4CT);

    return 0;
}
