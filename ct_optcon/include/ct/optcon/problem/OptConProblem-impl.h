/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::OptConProblem()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::OptConProblem(DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    LinearPtr_t linearSystem)
    : tf_(0.0),
      x0_(MANIFOLD::NeutralElement()),
      controlledSystem_(nonlinDynamics),
      costFunction_(costFunction),
      linearizedSystem_(linearSystem)
/*,
      inputBoxConstraints_(nullptr), // TODO: bring back this member
      stateBoxConstraints_(nullptr),
      generalConstraints_(nullptr) */
{
    if (linearSystem == nullptr)  // no linearization provided
    {
        linearizedSystem_ = std::shared_ptr<ct::core::SystemLinearizer<MANIFOLD, CONTROL_DIM, TIME_T>>(
            new ct::core::SystemLinearizer<MANIFOLD, CONTROL_DIM, TIME_T>(controlledSystem_));
    }
}


template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::OptConProblem(const Time_t tf,
    const MANIFOLD& x0,
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    LinearPtr_t linearSystem)
    : OptConProblem(nonlinDynamics, costFunction, linearSystem)  // delegating constructor
{
    tf_ = tf;
    x0_ = x0;
}

/*
template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::OptConProblem(DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    ConstraintPtr_t inputBoxConstraints,
    ConstraintPtr_t stateBoxConstraints,
    ConstraintPtr_t generalConstraints,
    LinearPtr_t linearSystem)
    : OptConProblem(nonlinDynamics, costFunction, linearSystem)  // delegating constructor
{
    inputBoxConstraints_ = inputBoxConstraints;
    stateBoxConstraints_ = stateBoxConstraints;
    generalConstraints_ = generalConstraints;
}


template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::OptConProblem(const Time_t tf,
    const MANIFOLD& x0,
    DynamicsPtr_t nonlinDynamics,
    CostFunctionPtr_t costFunction,
    ConstraintPtr_t inputBoxConstraints,
    ConstraintPtr_t stateBoxConstraints,
    ConstraintPtr_t generalConstraints,
    LinearPtr_t linearSystem)
    : OptConProblem(nonlinDynamics,
          costFunction,
          inputBoxConstraints,
          stateBoxConstraints,
          generalConstraints,
          linearSystem)  // delegating constructor
{
    tf_ = tf;
    x0_ = x0;
}
*/


template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::verify() const
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


template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::DynamicsPtr_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getNonlinearSystem() const
{
    return controlledSystem_;
}


template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::LinearPtr_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getLinearSystem() const
{
    return linearizedSystem_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::CostFunctionPtr_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getCostFunction() const
{
    return costFunction_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setNonlinearSystem(const DynamicsPtr_t dyn)
{
    controlledSystem_ = dyn;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setLinearSystem(const LinearPtr_t lin)
{
    linearizedSystem_ = lin;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setCostFunction(const CostFunctionPtr_t cost)
{
    costFunction_ = cost;
}

/*
template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setInputBoxConstraints(const ConstraintPtr_t constraint)
{
    inputBoxConstraints_ = constraint;
    if (!inputBoxConstraints_->isInitialized())
        inputBoxConstraints_->initialize();
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setStateBoxConstraints(const ConstraintPtr_t constraint)
{
    stateBoxConstraints_ = constraint;
    if (!stateBoxConstraints_->isInitialized())
        stateBoxConstraints_->initialize();
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setGeneralConstraints(const ConstraintPtr_t constraint)
{
    generalConstraints_ = constraint;
    if (!generalConstraints_->isInitialized())
        generalConstraints_->initialize();
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::ConstraintPtr_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getInputBoxConstraints() const
{
    return inputBoxConstraints_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::ConstraintPtr_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getStateBoxConstraints() const
{
    return stateBoxConstraints_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::ConstraintPtr_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getGeneralConstraints() const
{
    return generalConstraints_;
}
*/

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
const MANIFOLD OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getInitialState() const
{
    return x0_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setInitialState(const MANIFOLD& x0)
{
    x0_ = x0;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
typename OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::Time_t
OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::getTimeHorizon() const
{
    return tf_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, ct::core::TIME_TYPE TIME_T>
void OptConProblem<MANIFOLD, CONTROL_DIM, TIME_T>::setTimeHorizon(const Time_t tf)
{
    tf_ = tf;
}

}  // namespace optcon
}  // namespace ct
