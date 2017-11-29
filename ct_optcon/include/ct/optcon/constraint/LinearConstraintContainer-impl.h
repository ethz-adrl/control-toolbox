/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::LinearConstraintContainer()
    : initializedIntermediate_(false), initializedTerminal_(false)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::LinearConstraintContainer(
    const LinearConstraintContainer& arg)
    : ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      initializedIntermediate_(arg.initializedIntermediate_),
      initializedTerminal_(arg.initializedTerminal_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::~LinearConstraintContainer()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacNonZeroCount()
{
    return getJacobianStateNonZeroCountIntermediate() + getJacobianStateNonZeroCountTerminal() +
           getJacobianInputNonZeroCountIntermediate() + getJacobianInputNonZeroCountTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::initialize()
{
    initializedIntermediate_ = initializeIntermediate();
    initializedTerminal_ = initializeTerminal();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::isInitialized()
{
    return initializedIntermediate_ && initializedTerminal_;
}

}  // namespace optcon
}  // namespace ct
