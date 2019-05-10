/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptconContinuousSystemInterface(
    const optConProblem_t& problem,
    const settings_t& settings)
    : Base(problem, settings)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeNumStages(const int numStages)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeNonlinearSystem(
    const typename optConProblem_t::DynamicsPtr_t& dyn)
{
    if (dyn == nullptr)
        throw std::runtime_error("system dynamics are nullptr");

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        this->systems_.at(i) = typename optConProblem_t::DynamicsPtr_t(dyn->clone());
        this->systems_.at(i)->setController(this->controller_.at(i));

        discretizers_.at(i) = system_discretizer_ptr_t(new discretizer_t(
            this->systems_.at(i), this->settings_.dt, this->settings_.integrator, this->settings_.K_sim));
        discretizers_.at(i)->initialize();
    }
}
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeLinearSystem(
    const typename optConProblem_t::LinearPtr_t& lin)
{
    if (lin == nullptr)
        throw std::runtime_error("linear system dynamics are nullptr");

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        this->linearSystems_.at(i) = typename optConProblem_t::LinearPtr_t(lin->clone());
        sensitivity_.at(i)->setLinearSystem(this->linearSystems_.at(i));
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initialize()
{
    this->systems_.resize(this->settings_.nThreads + 1);
    this->linearSystems_.resize(this->settings_.nThreads + 1);
    discretizers_.resize(this->settings_.nThreads + 1);
    sensitivity_.resize(this->settings_.nThreads + 1);

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        // make a deep copy of the system for each thread
        this->systems_.at(i) =
            typename optConProblem_t::DynamicsPtr_t(this->optConProblem_.getNonlinearSystem()->clone());
        this->systems_.at(i)->setController(this->controller_.at(i));

        this->linearSystems_.at(i) =
            typename optConProblem_t::LinearPtr_t(this->optConProblem_.getLinearSystem()->clone());

        discretizers_.at(i) = system_discretizer_ptr_t(new discretizer_t(
            this->systems_.at(i), this->settings_.dt, this->settings_.integrator, this->settings_.K_sim));
        discretizers_.at(i)->initialize();

        if (this->settings_.useSensitivityIntegrator)
        {
            if (this->settings_.integrator != ct::core::IntegrationType::EULER &&
                this->settings_.integrator != ct::core::IntegrationType::EULERCT &&
                this->settings_.integrator != ct::core::IntegrationType::RK4 &&
                this->settings_.integrator != ct::core::IntegrationType::RK4CT &&
                this->settings_.integrator != ct::core::IntegrationType::EULER_SYM)
                throw std::runtime_error("sensitivity integrator only available for Euler and RK4 integrators");

            sensitivity_.at(i) = SensitivityPtr(
                new ct::core::SensitivityIntegrator<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>(
                    this->settings_.getSimulationTimestep(), this->linearSystems_.at(i), this->controller_.at(i),
                    this->settings_.integrator, this->settings_.timeVaryingDiscretization));
        }
        else
        {
            sensitivity_.at(i) = SensitivityPtr(
                new ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>(
                    this->settings_.dt, this->linearSystems_.at(i), this->settings_.discretization));
        }
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::configure(
    const settings_t& settings)
{
    if (settings.nThreads != this->settings_.nThreads)
    {
        throw std::runtime_error("Number of threads cannot be changed after instance has been created.");
    }
    if (settings.integrator != this->settings_.integrator)
    {
        throw std::runtime_error("Cannot change integration type.");
    }

    // update system discretizer with new settings
    for (int i = 0; i < settings.nThreads + 1; i++)
    {
        if (!sensitivity_[i])
            break;

        sensitivity_[i]->setApproximation(settings.discretization);

        if (settings.useSensitivityIntegrator)
            sensitivity_[i]->setTimeDiscretization(settings.getSimulationTimestep());
        else
            sensitivity_[i]->setTimeDiscretization(settings.dt);

        discretizers_[i]->setParameters(settings.dt, settings.K_sim);
    }

    this->settings_ = settings;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::propagateControlledDynamics(
    const state_vector_t& state,
    const time_t n,
    const control_vector_t& control,
    state_vector_t& stateNext,
    const size_t threadId)
{
    this->controller_[threadId]->setControl(control);
    discretizers_[threadId]->propagateControlledDynamics(state, n, control, stateNext);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSubstates(
    StateVectorArrayPtr& subStepsX,
    const size_t threadId)
{
    subStepsX = discretizers_[threadId]->getSubstates();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSubcontrols(
    ControlVectorArrayPtr& subStepsU,
    const size_t threadId)
{
    subStepsU = discretizers_[threadId]->getSubcontrols();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setSubstepTrajectoryReference(
    const StateSubstepsPtr& xSubsteps,
    const ControlSubstepsPtr& uSubsteps,
    const size_t threadId)
{
    sensitivity_[threadId]->setSubstepTrajectoryReference(xSubsteps.get(), uSubsteps.get());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void OptconContinuousSystemInterface<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getAandB(const state_vector_t& x,
    const control_vector_t& u,
    const state_vector_t& x_next,
    const int n,
    size_t subSteps,
    state_matrix_t& A,
    state_control_matrix_t& B,
    const size_t threadId)
{
    sensitivity_[threadId]->getAandB(x, u, x_next, n, subSteps, A, B);
}

}  // namespace optcon
}  // namespace ct
