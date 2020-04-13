/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::OptconDiscreteSystemInterface(const optConProblem_t& problem,
    const settings_t& settings)
    : Base(problem, settings)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::~OptconDiscreteSystemInterface()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::getAandB(const MANIFOLD& x,
    const control_vector_t& u,
    const MANIFOLD& x_next,
    const int n,
    size_t subSteps,
    state_matrix_t& A,
    state_control_matrix_t& B,
    const size_t threadId)
{
    this->linearSystems_[threadId]->getDerivatives(A, B, x, x_next, u, subSteps, n);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::computeControlledDynamics(const MANIFOLD& state,
    const int n,
    const control_vector_t& u,
    typename MANIFOLD::Tangent& dx,
    const size_t threadId)
{
    this->systems_[threadId]->propagateControlledDynamics(state, n, u, dx);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::changeNonlinearSystem(
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

template <typename MANIFOLD, size_t CONTROL_DIM>
void OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::changeLinearSystem(
    const typename optConProblem_t::LinearPtr_t& lin)
{
    if (lin == nullptr)
        throw std::runtime_error("linear system dynamics are nullptr");

    for (int i = 0; i < this->settings_.nThreads + 1; i++)
    {
        this->linearSystems_.at(i) = typename optConProblem_t::LinearPtr_t(lin->clone());
    }
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::initialize()
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

template <typename MANIFOLD, size_t CONTROL_DIM>
void OptconDiscreteSystemInterface<MANIFOLD, CONTROL_DIM>::configure(const settings_t& settings)
{
    if (settings.nThreads != this->settings_.nThreads)
    {
        throw std::runtime_error("Number of threads cannot be changed after instance has been created.");
    }

    this->settings_ = settings;
}

}  // namespace optcon
}  // namespace ct
