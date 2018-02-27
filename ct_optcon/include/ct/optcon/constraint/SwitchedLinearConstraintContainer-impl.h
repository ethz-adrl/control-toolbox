/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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
              activeLinearConstraintContainer_(switchedLinearConstraintContainers.front())
        {
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::SwitchedLinearConstraintContainer(
            const SwitchedLinearConstraintContainer& arg)
            : LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
              switchedLinearConstraintContainers_(arg.switchedLinearConstraintContainers_),
              continuousModeSequence_(arg.continuousModeSequence_),
              activeLinearConstraintContainer_(arg.activeLinearConstraintContainer_)
        {
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::~SwitchedLinearConstraintContainer()
        {
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::SwitchedLinearConstraintContainer_Raw_Ptr_t
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
        {
          auto clone_ = new SwitchedLinearConstraintContainer(*this);

          // Clone individual subsystems for thread safety
          clone_->switchedLinearConstraintContainers_.clear();
          for (auto& linearConstraintContainer : this->switchedLinearConstraintContainers_){
            clone_->switchedLinearConstraintContainers_.emplace_back(linearConstraintContainer->clone());
          }
          clone_->activeLinearConstraintContainer_ = clone_->switchedLinearConstraintContainers_.front();
          return clone_;
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
        {
            return activeLinearConstraintContainer_->evaluateIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
        {
            return switchedLinearConstraintContainers_.back()->evaluateTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateConstraintsCount()
        {
            return activeLinearConstraintContainer_->getIntermediateConstraintsCount();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getTerminalConstraintsCount()
        {
            return switchedLinearConstraintContainers_.back()->getTerminalConstraintsCount();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseIntermediate()
        {
            return activeLinearConstraintContainer_->jacobianStateSparseIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateIntermediate()
        {
            return activeLinearConstraintContainer_->jacobianStateIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseTerminal()
        {
            return switchedLinearConstraintContainers_.back()->jacobianStateSparseTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateTerminal()
        {
            return switchedLinearConstraintContainers_.back()->jacobianStateTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseIntermediate()
        {
            return activeLinearConstraintContainer_->jacobianInputSparseIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputIntermediate()
        {
            return activeLinearConstraintContainer_->jacobianInputIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseTerminal()
        {
            return switchedLinearConstraintContainers_.back()->jacobianInputSparseTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        typename SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
        SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputTerminal()
        {
            return switchedLinearConstraintContainers_.back()->jacobianInputTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols)
        {
            return activeLinearConstraintContainer_->sparsityPatternStateIntermediate(iRows, jCols);
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols)
        {
            return switchedLinearConstraintContainers_.back()->sparsityPatternStateTerminal(iRows, jCols);
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols)
        {
            return activeLinearConstraintContainer_->sparsityPatternInputIntermediate(iRows, jCols);
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols)
        {
            return switchedLinearConstraintContainers_.back()->sparsityPatternInputTerminal(iRows, jCols);
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountIntermediate()
        {
            return activeLinearConstraintContainer_->getJacobianStateNonZeroCountIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountTerminal()
        {
            return switchedLinearConstraintContainers_.back()->getJacobianStateNonZeroCountTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountIntermediate()
        {
            return activeLinearConstraintContainer_->getJacobianInputNonZeroCountIntermediate();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        size_t SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountTerminal()
        {
            return switchedLinearConstraintContainers_.back()->getJacobianInputNonZeroCountTerminal();
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        bool SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::initializeIntermediate()
        {
            for (auto& linearConstraintContainer : switchedLinearConstraintContainers_){
                if (~linearConstraintContainer->initializeIntermediate()){
                    return false;
                }
            }
            return true;
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        bool SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::initializeTerminal()
        {
          if (switchedLinearConstraintContainers_.back()->initializeTerminal()) {
            this->lowerBoundsTerminal_ = switchedLinearConstraintContainers_.back()->getLowerBoundsTerminal();
            this->upperBoundsTerminal_ = switchedLinearConstraintContainers_.back()->getUpperBoundsTerminal();
            return true;
          }
          return false;
        };

        template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
        void SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>::update()
        {
            auto mode = continuousModeSequence_.getPhaseFromTime(this->t_);
            activeLinearConstraintContainer_ = switchedLinearConstraintContainers_[mode];
            activeLinearConstraintContainer_->setCurrentStateAndControl(this->x_, this->u_, this->t_);

          this->lowerBoundsIntermediate_ = activeLinearConstraintContainer_->getLowerBoundsIntermediate();
          this->upperBoundsIntermediate_ = activeLinearConstraintContainer_->getUpperBoundsIntermediate();
        };


    }  // namespace optcon
}  // namespace ct
