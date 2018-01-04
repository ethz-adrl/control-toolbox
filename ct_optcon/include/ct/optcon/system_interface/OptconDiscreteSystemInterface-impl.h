/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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
    throw std::runtime_error("Not yet implemented");
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
    throw std::runtime_error("Not yet implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::propagateControlledDynamics(
    const state_vector_t& state,
    const time_t n,
    const control_vector_t& control,
    state_vector_t& stateNext,
    const size_t threadId)
{
    throw std::runtime_error("Not yet implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::changeNonlinearSystem(
    const typename optConProblem_t::DynamicsPtr_t& dyn)
{
    throw std::runtime_error("Not yet implemented");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptconDiscreteSystemInterface<STATE_DIM, CONTROL_DIM, SCALAR>::changeLinearSystem(
    const typename optConProblem_t::LinearPtr_t& lin)
{
    throw std::runtime_error("Not yet implemented");
}

}  // namespace optcon
}  // namespace ct
