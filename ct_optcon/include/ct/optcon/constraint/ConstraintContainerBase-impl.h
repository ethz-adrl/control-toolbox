/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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


}  // namespace optcon
}  // namespace ct
