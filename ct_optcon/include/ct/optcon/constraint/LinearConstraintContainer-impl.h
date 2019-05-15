/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::printout()
{
    Eigen::VectorXi iRows, jCols;  // local var

    std::cout << std::endl;

    std::cout << "LinearConstraintContainer - Printout INTERMEDIATE STAGE" << std::endl;

    std::cout << "getLowerBoundsIntermediate().transpose() :" << std::endl;
    std::cout << this->getLowerBoundsIntermediate().transpose() << std::endl;

    std::cout << "getUpperBoundsIntermediate().transpose() :" << std::endl;
    std::cout << this->getUpperBoundsIntermediate().transpose() << std::endl;

    std::cout << "getJacobianStateNonZeroCountIntermediate() :" << std::endl;
    std::cout << getJacobianStateNonZeroCountIntermediate() << std::endl;

    std::cout << "getJacobianInputNonZeroCountIntermediate() :" << std::endl;
    std::cout << getJacobianInputNonZeroCountIntermediate() << std::endl;

    std::cout << "jacobianStateIntermediate() :" << std::endl;
    std::cout << jacobianStateIntermediate() << std::endl;

    std::cout << "jacobianStateSparseIntermediate() :" << std::endl;
    std::cout << jacobianStateSparseIntermediate() << std::endl;

    std::cout << "jacobianInputIntermediate() :" << std::endl;
    std::cout << jacobianInputIntermediate() << std::endl;

    std::cout << "jacobianInputSparseIntermediate() :" << std::endl;
    std::cout << jacobianInputSparseIntermediate() << std::endl;

    std::cout << "sparsityPatternStateIntermediate(iRows, jCols) :" << std::endl;
    sparsityPatternStateIntermediate(iRows, jCols);
    std::cout << "iRows: " << iRows.transpose() << std::endl;
    std::cout << "jCols: " << jCols.transpose() << std::endl;

    std::cout << "sparsityPatternInputIntermediate(iRows, jCols) :" << std::endl;
    sparsityPatternInputIntermediate(iRows, jCols);
    std::cout << "iRows: " << iRows.transpose() << std::endl;
    std::cout << "jCols: " << jCols.transpose() << std::endl;


    std::cout << "LinearConstraintContainer - Printout TERMINAL STAGE" << std::endl;

    std::cout << "getLowerBoundsTerminal().transpose() :" << std::endl;
    std::cout << this->getLowerBoundsTerminal().transpose() << std::endl;

    std::cout << "getUpperBoundsTerminal().transpose() :" << std::endl;
    std::cout << this->getUpperBoundsTerminal().transpose() << std::endl;

    std::cout << "getJacobianStateNonZeroCountTerminal() :" << std::endl;
    std::cout << getJacobianStateNonZeroCountTerminal() << std::endl;

    std::cout << "getJacobianInputNonZeroCountTerminal() :" << std::endl;
    std::cout << getJacobianInputNonZeroCountTerminal() << std::endl;

    std::cout << "jacobianStateTerminal() :" << std::endl;
    std::cout << jacobianStateTerminal() << std::endl;

    std::cout << "jacobianStateSparseTerminal() :" << std::endl;
    std::cout << jacobianStateSparseTerminal() << std::endl;

    std::cout << "jacobianInputTerminal() :" << std::endl;
    std::cout << jacobianInputTerminal() << std::endl;

    std::cout << "jacobianInputSparseTerminal() :" << std::endl;
    std::cout << jacobianInputSparseTerminal() << std::endl;

    std::cout << "sparsityPatternStateTerminal(iRows, jCols) :" << std::endl;
    sparsityPatternStateTerminal(iRows, jCols);
    std::cout << "iRows: " << iRows.transpose() << std::endl;
    std::cout << "jCols: " << jCols.transpose() << std::endl;

    std::cout << "sparsityPatternInputTerminal(iRows, jCols) :" << std::endl;
    sparsityPatternInputTerminal(iRows, jCols);
    std::cout << "iRows: " << iRows.transpose() << std::endl;
    std::cout << "jCols: " << jCols.transpose() << std::endl;

    std::cout << std::endl;
    std::cout << std::endl;
}


}  // namespace optcon
}  // namespace ct
