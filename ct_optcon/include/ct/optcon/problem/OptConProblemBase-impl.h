/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::OptConProblemBase(
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    LinearPtr_t linearSystem)
    : tf_(0.0),
      x0_(state_vector_t::Zero()),
      controlledSystem_(nonlinDynamics),
      costFunction_(costFunction),
      linearizedSystem_(linearSystem),
      inputBoxConstraints_(nullptr),
      stateBoxConstraints_(nullptr),
      generalConstraints_(nullptr)
{
    if (linearSystem == nullptr)  // no linearization provided
    {
        linearizedSystem_ = std::shared_ptr<LINEARIZER_T>(new LINEARIZER_T(controlledSystem_));
    }
}


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::OptConProblemBase(
    const time_t tf,
    const state_vector_t& x0,
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    LinearPtr_t linearSystem)
    : OptConProblemBase(nonlinDynamics, costFunction, linearSystem)  // delegating constructor
{
    tf_ = tf;
    x0_ = x0;
}


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::OptConProblemBase(
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    ConstraintPtr_t inputBoxConstraints,
    ConstraintPtr_t stateBoxConstraints,
    ConstraintPtr_t generalConstraints,
    LinearPtr_t linearSystem)
    : OptConProblemBase(nonlinDynamics, costFunction, linearSystem)  // delegating constructor
{
    inputBoxConstraints_ = inputBoxConstraints;
    stateBoxConstraints_ = stateBoxConstraints;
    generalConstraints_ = generalConstraints;
}


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::OptConProblemBase(
    const time_t tf,
    const state_vector_t& x0,
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    ConstraintPtr_t inputBoxConstraints,
    ConstraintPtr_t stateBoxConstraints,
    ConstraintPtr_t generalConstraints,
    LinearPtr_t linearSystem)
    : OptConProblemBase(nonlinDynamics,
          costFunction,
          inputBoxConstraints,
          stateBoxConstraints,
          generalConstraints,
          linearSystem)  // delegating constructor
{
    tf_ = tf;
    x0_ = x0;
}


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::verify() const
{
    if (!controlledSystem_)
    {
        throw std::runtime_error("Dynamic system not set");
    }
    if (!linearizedSystem_)
    {
        throw std::runtime_error("Linearized system not set");
    }
    if (!costFunction_)
    {
        throw std::runtime_error("Cost function not set");
    }
    if (tf_ < 0.0)
    {
        throw std::runtime_error("Time horizon should not be negative");
    }
}


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::DynamicsPtr_t
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getNonlinearSystem() const
{
    return controlledSystem_;
}


template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::LinearPtr_t
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getLinearSystem() const
{
    return linearizedSystem_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::
    CostFunctionPtr_t
    OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getCostFunction() const
{
    return costFunction_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setNonlinearSystem(
    const DynamicsPtr_t dyn)
{
    controlledSystem_ = dyn;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setLinearSystem(
    const LinearPtr_t lin)
{
    linearizedSystem_ = lin;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setCostFunction(
    const CostFunctionPtr_t cost)
{
    costFunction_ = cost;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setInputBoxConstraints(
    const ConstraintPtr_t constraint)
{
    inputBoxConstraints_ = constraint;
    if (!inputBoxConstraints_->isInitialized())
        inputBoxConstraints_->initialize();
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setStateBoxConstraints(
    const ConstraintPtr_t constraint)
{
    stateBoxConstraints_ = constraint;
    if (!stateBoxConstraints_->isInitialized())
        stateBoxConstraints_->initialize();
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setGeneralConstraints(
    const ConstraintPtr_t constraint)
{
    generalConstraints_ = constraint;
    if (!generalConstraints_->isInitialized())
        generalConstraints_->initialize();
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::
    ConstraintPtr_t
    OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getInputBoxConstraints()
        const
{
    return inputBoxConstraints_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::
    ConstraintPtr_t
    OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getStateBoxConstraints()
        const
{
    return stateBoxConstraints_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::
    ConstraintPtr_t
    OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getGeneralConstraints()
        const
{
    return generalConstraints_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
const typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::
    state_vector_t
    OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getInitialState() const
{
    return x0_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setInitialState(
    const state_vector_t& x0)
{
    x0_ = x0;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
typename OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::time_t
OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::getTimeHorizon() const
{
    return tf_;
}

template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    typename SYSTEM_T,
    typename LINEAR_SYSTEM_T,
    typename LINEARIZER_T,
    typename SCALAR>
void OptConProblemBase<STATE_DIM, CONTROL_DIM, SYSTEM_T, LINEAR_SYSTEM_T, LINEARIZER_T, SCALAR>::setTimeHorizon(
    const time_t tf)
{
    tf_ = tf;
}


}  // namespace optcon
}  // namespace ct
