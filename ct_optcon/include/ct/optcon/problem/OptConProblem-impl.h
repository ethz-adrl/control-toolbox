/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
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
      stateInputConstraints_(nullptr),
      pureStateConstraints_(nullptr)
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
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setStateInputConstraints(const ConstraintPtr_t constraint)
{
    stateInputConstraints_ = constraint;
    if (!stateInputConstraints_->isInitialized())
        stateInputConstraints_->initialize();
    if ((stateInputConstraints_->getJacobianInputNonZeroCountIntermediate() +
            stateInputConstraints_->getJacobianInputNonZeroCountTerminal()) == 0)
        std::cout << "WARNING: The state input constraint container does not"
                  << " contain any elements in the constraint jacobian with respect to the input."
                  << " Consider adding the constraints as pure state constraints. " << std::endl;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setPureStateConstraints(const ConstraintPtr_t constraint)
{
    pureStateConstraints_ = constraint;
    if (!pureStateConstraints_->isInitialized())
        pureStateConstraints_->initialize();
    if ((pureStateConstraints_->getJacobianInputNonZeroCountIntermediate() +
            pureStateConstraints_->getJacobianInputNonZeroCountTerminal()) > 0)
        throw std::runtime_error(
            "Pure state constraints contain an element with a non zero derivative with respect to control input."
            " Implement this constraint as state input constraint");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getStateInputConstraints() const
{
    return stateInputConstraints_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintPtr_t
OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getPureStateConstraints() const
{
    return pureStateConstraints_;
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
