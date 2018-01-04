/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::OptConProblem()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::OptConProblem(DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    LinearPtr_t linearSystem)
    : tf_(0.0),
      x0_(state_vector_t::Zero()),
      controlledSystem_(nonlinDynamics),
      costFunction_(costFunction),
      linearizedSystem_(linearSystem),
      boxConstraints_(nullptr),
      generalConstraints_(nullptr)
{
    if (linearSystem == nullptr)  // no linearization provided
    {
        linearizedSystem_ = std::shared_ptr<core::SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new core::SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>(controlledSystem_));
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::OptConProblem(const SCALAR& tf,
    const state_vector_t& x0,
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    LinearPtr_t linearSystem)
    : OptConProblem(nonlinDynamics, costFunction, linearSystem)  // delegating constructor
{
    tf_ = tf;
    x0_ = x0;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::OptConProblem(DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    ConstraintPtr_t boxConstraints,
    ConstraintPtr_t generalConstraints,
    LinearPtr_t linearSystem)
    : OptConProblem(nonlinDynamics, costFunction, linearSystem)  // delegating constructor
{
    boxConstraints_ = boxConstraints;
    generalConstraints_ = generalConstraints;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::OptConProblem(const SCALAR& tf,
    const state_vector_t& x0,
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    ConstraintPtr_t boxConstraints,
    ConstraintPtr_t generalConstraints,
    LinearPtr_t linearSystem)
    : OptConProblem(nonlinDynamics,
          costFunction,
          boxConstraints,
          generalConstraints,
          linearSystem)  // delegating constructor
{
    tf_ = tf;
    x0_ = x0;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::verify() const
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


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::DynamicsPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getNonlinearSystem() const
{
    return controlledSystem_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::LinearPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getLinearSystem() const
{
    return linearizedSystem_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getCostFunction() const
{
    return costFunction_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setNonlinearSystem(const DynamicsPtr_t dyn)
{
    controlledSystem_ = dyn;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setLinearSystem(const LinearPtr_t lin)
{
    linearizedSystem_ = lin;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setCostFunction(const CostFunctionPtr_t cost)
{
    costFunction_ = cost;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setBoxConstraints(const ConstraintPtr_t constraint)
{
    boxConstraints_ = constraint;
    if (!boxConstraints_->isInitialized())
        boxConstraints_->initialize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setGeneralConstraints(const ConstraintPtr_t constraint)
{
    generalConstraints_ = constraint;
    if (!generalConstraints_->isInitialized())
        generalConstraints_->initialize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getBoxConstraints() const
{
    return boxConstraints_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getGeneralConstraints() const
{
    return generalConstraints_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getInitialState() const
{
    return x0_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setInitialState(const state_vector_t& x0)
{
    x0_ = x0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const SCALAR& OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getTimeHorizon() const
{
    return tf_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setTimeHorizon(const SCALAR& tf)
{
    tf_ = tf;
}


}  // optcon
}  // ct
