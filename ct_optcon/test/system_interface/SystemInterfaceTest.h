/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This file implements a unit tests for the OptconSystemInterface classes
 */

#pragma once

#include <gtest/gtest.h>

#include "../testSystems/LinearOscillator.h"

namespace ct {
namespace optcon {
namespace example {

//TODO test initializing from OptConProblem

TEST(SystemInterfaceTest, ContinuousSystemInterface)
{
    const size_t STATE_DIM = state_dim;
    const size_t CONTROL_DIM = control_dim;
    const size_t P_DIM = STATE_DIM / 2;
    const size_t V_DIM = STATE_DIM / 2;
    typedef double SCALAR;

    typedef ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;

    std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM>> nonlinearSystem(new LinearOscillator());

    std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM>> analyticLinearSystem(new LinearOscillatorLinear());

    Eigen::Vector2d x_final;
    x_final << 1.0, 0.0;

    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
        tpl::createCostFunctionLinearOscillator<SCALAR>(x_final);

    // fill minimal version of optconproblem
    OptConProblem_t optConProblem(nonlinearSystem, costFunction, analyticLinearSystem);

    typedef OptconSystemInterface<STATE_DIM, CONTROL_DIM, OptConProblem_t, SCALAR> systemInterface_t;
    typedef std::shared_ptr<systemInterface_t> systemInterfacePtr_t;

    // settings
    NLOptConSettings nloc_settings;
    nloc_settings.dt = 0.01;

    // toggle sensitivity integrator
    for (size_t sensInt = 0; sensInt <= 1; sensInt++)
    {
        nloc_settings.useSensitivityIntegrator = bool(sensInt);

        // toggle over simulation time-steps
        for (size_t ksim = 1; ksim <= 5; ksim = ksim + 4)
        {
            nloc_settings.K_sim = ksim;

            // catch special case, simulation sub-time steps only make sense when sensitivity integrator active
            if ((nloc_settings.useSensitivityIntegrator == false) && (ksim > 1))
                continue;  // proceed to next test case

            // toggle integrator type
            for (size_t integratortype = 0; integratortype <= 1; integratortype++)
            {
                if (integratortype == 0)
                    nloc_settings.integrator = ct::core::IntegrationType::EULERCT;
                else if (integratortype == 1 && nloc_settings.useSensitivityIntegrator == true)
                {
                    // use RK4 with exactly integrated sensitivities
                    nloc_settings.integrator = ct::core::IntegrationType::RK4CT;
                }
                else
                    continue;  // proceed to next test case


                // nloc_settings.print();

                systemInterfacePtr_t systemInterface(
                    new OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(
                        optConProblem, nloc_settings));
                systemInterface->initialize();

                // propagate the dynamics inside systemInterface and find sensitivities
                systemInterface_t::state_vector_t x_start, x_next;
                x_start.setRandom();
                systemInterface_t::control_vector_t control;
                control.setRandom();

                systemInterface_t::state_matrix_t A;
                systemInterface_t::state_control_matrix_t B;

                systemInterface_t::StateSubstepsPtr subStepsX(new systemInterface_t::StateSubsteps());
                systemInterface_t::ControlSubstepsPtr subStepsU(new systemInterface_t::ControlSubsteps());
                subStepsX->resize(1);
                subStepsU->resize(1);

                systemInterface->propagateControlledDynamics(x_start, 0, control, x_next, 0);
                systemInterface->getSubstates(subStepsX->at(0), 0);
                std::cout << "subStepsX->at(0) with K_sim " << nloc_settings.K_sim << std::endl;
                for(const auto& substep : *(subStepsX->at(0))){
                  std::cout << substep.transpose() << std::endl;
                }
                systemInterface->getSubcontrols(subStepsU->at(0), 0);
                std::cout << "subStepsU->at(0) with K_sim " << nloc_settings.K_sim << std::endl;
                for(const auto& substep : *(subStepsU->at(0))){
                  std::cout << substep.transpose() << std::endl;
                  ASSERT_TRUE(substep.isApprox(control, 1e-10)); // constant controller
                }
                systemInterface->setSubstepTrajectoryReference(subStepsX, subStepsU, 0);
                systemInterface->getAandB(x_start, control, x_next, 0, nloc_settings.K_sim, A, B, 0);

                // use discretizer and check against systemInterface
                typedef ct::core::SystemDiscretizer<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR> discretizer_t;
                discretizer_t discretizer(
                    nonlinearSystem, nloc_settings.dt, nloc_settings.integrator, nloc_settings.K_sim);
                discretizer.initialize();

                systemInterface_t::state_vector_t x_next_discretizer;
                discretizer.propagateControlledDynamics(x_start, 0, control, x_next_discretizer);

                ASSERT_TRUE(x_next.isApprox(x_next_discretizer, 1e-8));

                // use sensitivity and check against systemInterface
                typedef ct::core::Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR> Sensitivity_t;
                typedef std::shared_ptr<Sensitivity_t> SensitivityPtr;
                SensitivityPtr sensitivity;

                typedef ct::core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR> constant_controller_t;
                typedef std::shared_ptr<constant_controller_t> ConstantControllerPtr;
                ConstantControllerPtr controller(new constant_controller_t());

                if (nloc_settings.useSensitivityIntegrator)
                {
                    if (nloc_settings.integrator != ct::core::IntegrationType::EULER &&
                        nloc_settings.integrator != ct::core::IntegrationType::EULERCT &&
                        nloc_settings.integrator != ct::core::IntegrationType::RK4 &&
                        nloc_settings.integrator != ct::core::IntegrationType::RK4CT &&
                        nloc_settings.integrator != ct::core::IntegrationType::EULER_SYM)
                        throw std::runtime_error("sensitivity integrator only available for Euler and RK4 integrators");

                    sensitivity.reset(new ct::core::SensitivityIntegrator<STATE_DIM, CONTROL_DIM, STATE_DIM / 2,
                        STATE_DIM / 2, SCALAR>(nloc_settings.getSimulationTimestep(), analyticLinearSystem, controller,
                        nloc_settings.integrator, nloc_settings.timeVaryingDiscretization));
                    //TODO double check simulationTimeStep vs dt
                }
                else
                {
                    sensitivity.reset(new ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2,
                        STATE_DIM / 2, SCALAR>(nloc_settings.dt, analyticLinearSystem, nloc_settings.discretization));
                }

                systemInterface_t::state_matrix_t A_sensitivity;
                systemInterface_t::state_control_matrix_t B_sensitivity;
                sensitivity->setSubstepTrajectoryReference(subStepsX.get(), subStepsU.get());
                sensitivity->getAandB(x_start, control, x_next, 0, nloc_settings.K_sim, A_sensitivity, B_sensitivity);

                ASSERT_TRUE(A.isApprox(A_sensitivity, 1e-8));
                ASSERT_TRUE(B.isApprox(B_sensitivity, 1e-8));
            }
        }
    }
}

TEST(SystemInterfaceTest, DiscreteSystemInterface)
{
    //TODO
}

}  // namespace example
}  // namespace optcon
}  // namespace ct
