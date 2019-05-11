/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::SwitchedLinearConstraintContainer(
    const SwitchedLinearConstraintContainers& switchedLinearConstraintContainers,
    const ModeSequence_t& continuousModeSequence)
    : switchedLinearConstraintContainers_(switchedLinearConstraintContainers),
      continuousModeSequence_(continuousModeSequence),
      activeLinearConstraintContainer_(switchedLinearConstraintContainers.front()),
      terminalLinearConstraintContainer_(switchedLinearConstraintContainers.at(continuousModeSequence.getFinalPhase()))
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::SwitchedLinearConstraintContainer(
    const SwitchedLinearConstraintContainer& arg)
    : LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      continuousModeSequence_(arg.continuousModeSequence_)
{
    // Clone individual constraints
    switchedLinearConstraintContainers_.clear();
    for (auto& linearConstraintContainer : arg.switchedLinearConstraintContainers_)
    {
        switchedLinearConstraintContainers_.emplace_back(linearConstraintContainer->clone());
    }

    activeLinearConstraintContainer_ = switchedLinearConstraintContainers_.front();
    terminalLinearConstraintContainer_ =
        switchedLinearConstraintContainers_.at(continuousModeSequence_.getFinalPhase());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::~SwitchedLinearConstraintContainer()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::SwitchedLinearConstraintContainer_Raw_Ptr_t
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new SwitchedLinearConstraintContainer(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
    return activeLinearConstraintContainer_->evaluateIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
    return terminalLinearConstraintContainer_->evaluateTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateConstraintsCount()
{
    return activeLinearConstraintContainer_->getIntermediateConstraintsCount();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getTerminalConstraintsCount()
{
    return terminalLinearConstraintContainer_->getTerminalConstraintsCount();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseIntermediate()
{
    return activeLinearConstraintContainer_->jacobianStateSparseIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateIntermediate()
{
    return activeLinearConstraintContainer_->jacobianStateIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseTerminal()
{
    return terminalLinearConstraintContainer_->jacobianStateSparseTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateTerminal()
{
    return terminalLinearConstraintContainer_->jacobianStateTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseIntermediate()
{
    return activeLinearConstraintContainer_->jacobianInputSparseIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputIntermediate()
{
    return activeLinearConstraintContainer_->jacobianInputIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseTerminal()
{
    return terminalLinearConstraintContainer_->jacobianInputSparseTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputTerminal()
{
    return terminalLinearConstraintContainer_->jacobianInputTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateIntermediate(
    Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    return activeLinearConstraintContainer_->sparsityPatternStateIntermediate(iRows, jCols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateTerminal(
    Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    return terminalLinearConstraintContainer_->sparsityPatternStateTerminal(iRows, jCols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputIntermediate(
    Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    return activeLinearConstraintContainer_->sparsityPatternInputIntermediate(iRows, jCols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputTerminal(
    Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    return terminalLinearConstraintContainer_->sparsityPatternInputTerminal(iRows, jCols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountIntermediate()
{
    return activeLinearConstraintContainer_->getJacobianStateNonZeroCountIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountTerminal()
{
    return terminalLinearConstraintContainer_->getJacobianStateNonZeroCountTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountIntermediate()
{
    return activeLinearConstraintContainer_->getJacobianInputNonZeroCountIntermediate();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountTerminal()
{
    return terminalLinearConstraintContainer_->getJacobianInputNonZeroCountTerminal();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::initializeIntermediate()
{
    for (auto linearConstraintContainer : switchedLinearConstraintContainers_)
    {
        linearConstraintContainer->initialize();
        if (!linearConstraintContainer->isInitialized())
        {
            std::cout << "SwitchedLinearConstraintContainer::initializeIntermediate(): "
                      << "one of the constraints failed to initialize" << std::endl;
            return false;
        }
    }
    return true;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::initializeTerminal()
{
    // All in switchedLinearConstraintContainers_ already initialized in initializeIntermediate()
    this->lowerBoundsTerminal_ = terminalLinearConstraintContainer_->getLowerBoundsTerminal();
    this->upperBoundsTerminal_ = terminalLinearConstraintContainer_->getUpperBoundsTerminal();
    return true;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::update()
{
    auto mode = continuousModeSequence_.getPhaseFromTime(this->t_);
    activeLinearConstraintContainer_ = switchedLinearConstraintContainers_.at(mode);
    activeLinearConstraintContainer_->setCurrentStateAndControl(this->x_, this->u_, this->t_);

    this->lowerBoundsIntermediate_ = activeLinearConstraintContainer_->getLowerBoundsIntermediate();
    this->upperBoundsIntermediate_ = activeLinearConstraintContainer_->getUpperBoundsIntermediate();
}

}  // namespace optcon
}  // namespace ct
