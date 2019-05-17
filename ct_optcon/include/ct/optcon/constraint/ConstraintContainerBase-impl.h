/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerBase()
    : x_(state_vector_t::Zero()), u_(input_vector_t::Zero()), t_(SCALAR(0.0))
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerBase(const ConstraintContainerBase& arg)
    : x_(arg.x_),
      u_(arg.u_),
      t_(arg.t_),
      lowerBoundsIntermediate_(arg.lowerBoundsIntermediate_),
      lowerBoundsTerminal_(arg.lowerBoundsTerminal_),
      upperBoundsIntermediate_(arg.upperBoundsIntermediate_),
      upperBoundsTerminal_(arg.upperBoundsTerminal_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::~ConstraintContainerBase()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
    const input_vector_t& u,
    const SCALAR t)
{
    t_ = t;
    x_ = x;
    u_ = u;
    update();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintsCount()
{
    return getIntermediateConstraintsCount() + getTerminalConstraintsCount();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBoundsIntermediate() const
{
    return lowerBoundsIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBoundsTerminal() const
{
    return lowerBoundsTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBoundsIntermediate() const
{
    return upperBoundsIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBoundsTerminal() const
{
    return upperBoundsTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBoundsViolationIntermediate()
{
    const VectorXs vZero = VectorXs::Zero(upperBoundsIntermediate_.rows());
    return (evaluateIntermediate() - upperBoundsIntermediate_).array().max(vZero.array());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBoundsViolationIntermediate()
{
    const VectorXs vZero = VectorXs::Zero(lowerBoundsIntermediate_.rows());
    return (evaluateIntermediate() - lowerBoundsIntermediate_).array().min(vZero.array());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBoundsViolationTerminal()
{
    const VectorXs vZero = VectorXs::Zero(upperBoundsTerminal_.rows());
    return (evaluateTerminal() - upperBoundsTerminal_).array().max(vZero.array());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBoundsViolationTerminal()
{
    const VectorXs vZero = VectorXs::Zero(lowerBoundsTerminal_.rows());
    return (evaluateTerminal() - lowerBoundsTerminal_).array().min(vZero.array());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getTotalBoundsViolationIntermediate()
{
    const VectorXs vZero = VectorXs::Zero(lowerBoundsIntermediate_.rows());
    VectorXs eval = evaluateIntermediate();
    return (eval - lowerBoundsIntermediate_).array().min(vZero.array()) +
           (eval - upperBoundsIntermediate_).array().max(vZero.array());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getTotalBoundsViolationTerminal()
{
    const VectorXs vZero = VectorXs::Zero(lowerBoundsTerminal_.rows());
    VectorXs eval = evaluateTerminal();
    return (eval - lowerBoundsTerminal_).array().min(vZero.array()) +
           (eval - upperBoundsTerminal_).array().max(vZero.array());
}

}  // namespace optcon
}  // namespace ct
