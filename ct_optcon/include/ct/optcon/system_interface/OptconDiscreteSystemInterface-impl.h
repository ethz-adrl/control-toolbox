/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::OptconDiscreteSystemInterface(
    const optConProblem_t& problem,
    const settings_t& settings)
    : Base(problem, settings)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::getAandB(const state_vector_t& x,
    const control_vector_t& u,
    const state_vector_t& x_next,
    const int n,
    size_t subSteps,
    state_matrix_t& A,
    state_control_matrix_t& B,
    const size_t threadId)
{
    this->linearSystems_[threadId]->getAandB(x, u, x_next, n, subSteps, A, B);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::propagateControlledDynamics(
    const state_vector_t& state,
    const time_t n,
    const control_vector_t& control,
    state_vector_t& stateNext,
    const size_t threadId)
{
    this->controller_[threadId]->setControl(control);
    this->systems_[threadId]->propagateControlledDynamics(state, n, control, stateNext);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::changeNonlinearSystem(
    const typename optConProblem_t::DynamicsPtr_t& dyn)
{
    if (dyn == nullptr)
        throw std::runtime_error("system dynamics are nullptr");

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        this->systems_.at(i) = typename optConProblem_t::DynamicsPtr_t(dyn->clone());
        this->systems_.at(i)->setController(this->controller_.at(i));
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::changeLinearSystem(
    const typename optConProblem_t::LinearPtr_t& lin)
{
    if (lin == nullptr)
        throw std::runtime_error("linear system dynamics are nullptr");

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        this->linearSystems_.at(i) = typename optConProblem_t::LinearPtr_t(lin->clone());
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::initialize()
{
    this->systems_.resize(this->settings_.nThreads + 1);
    this->linearSystems_.resize(this->settings_.nThreads + 1);

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        // make a deep copy of the system for each thread
        this->systems_.at(i) =
            typename optConProblem_t::DynamicsPtr_t(this->optConProblem_.getNonlinearSystem()->clone());
        this->systems_.at(i)->setController(this->controller_.at(i));

        this->linearSystems_.at(i) =
            typename optConProblem_t::LinearPtr_t(this->optConProblem_.getLinearSystem()->clone());
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::configure(const settings_t& settings)
{
    if (settings.nThreads != this->settings_.nThreads)
    {
        throw std::runtime_error("Number of threads cannot be changed after instance has been created.");
    }

    this->settings_ = settings;
}

}  // namespace optcon
}  // namespace ct
