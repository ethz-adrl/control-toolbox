/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/constraint/ConstraintContainerAD.h>
#include <ct/optcon/nlp/DiscreteConstraintBase.h>

#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/TimeGrid.h>
#include <ct/optcon/dms/dms_core/spline/SplinerBase.h>

namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      The class takes continuous constraints defined with the
 *             constraint toolbox and discretizes them over the DMS shots. These
 *             discretized constraints can then be used in the NLP module
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintDiscretizer : public tpl::DiscreteConstraintBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef tpl::DiscreteConstraintBase<SCALAR> BASE;
    typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;

    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;

    typedef ct::core::tpl::TimeArray<SCALAR> time_array_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;

    /**
	 * @brief      Default constructor
	 */
    ConstraintDiscretizer() {}
    /**
	 * @brief      Destructor
	 */
    virtual ~ConstraintDiscretizer() {}
    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  w             The optimization variables
	 * @param[in]  c_continuous  The continuous constraints
	 * @param[in]  activeInd     A vector defining at which shots the constraint
	 *                           is active
	 */
    ConstraintDiscretizer(std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
        std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner,
        std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid,
        size_t N)
        : w_(w),
          controlSpliner_(controlSpliner),
          timeGrid_(timeGrid),
          N_(N),
          constraintsCount_(0),
          constraintsIntermediateCount_(0),
          constraintsTerminalCount_(0),
          nonZeroJacCount_(0),
          nonZeroJacCountIntermediate_(0),
          nonZeroJacCountTerminal_(0)
    {
    }


    void setBoxConstraints(std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> boxConstraints)
    {
        constraints_.push_back(boxConstraints);
        constraintsIntermediateCount_ += (N_ + 1) * boxConstraints->getIntermediateConstraintsCount();
        constraintsTerminalCount_ += boxConstraints->getTerminalConstraintsCount();
        constraintsCount_ = constraintsIntermediateCount_ + constraintsTerminalCount_;

        discreteConstraints_.resize(constraintsCount_);
        discreteLowerBound_.resize(constraintsCount_);
        discreteUpperBound_.resize(constraintsCount_);

        nonZeroJacCountIntermediate_ += (N_ + 1) * (boxConstraints->getJacobianStateNonZeroCountIntermediate() +
                                                       boxConstraints->getJacobianInputNonZeroCountIntermediate());
        nonZeroJacCountTerminal_ += boxConstraints->getJacobianStateNonZeroCountTerminal() +
                                    boxConstraints->getJacobianInputNonZeroCountTerminal();
        nonZeroJacCount_ = nonZeroJacCountIntermediate_ + nonZeroJacCountTerminal_;

        discreteJac_.resize(nonZeroJacCount_);
        discreteIRow_.resize(nonZeroJacCount_);
        discreteJCol_.resize(nonZeroJacCount_);
    }

    void setGeneralConstraints(
        std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> generalConstraints)
    {
        constraints_.push_back(generalConstraints);
        constraintsIntermediateCount_ += (N_ + 1) * generalConstraints->getIntermediateConstraintsCount();
        constraintsTerminalCount_ += generalConstraints->getTerminalConstraintsCount();
        constraintsCount_ = constraintsIntermediateCount_ + constraintsTerminalCount_;

        discreteConstraints_.resize(constraintsCount_);
        discreteLowerBound_.resize(constraintsCount_);
        discreteUpperBound_.resize(constraintsCount_);

        nonZeroJacCountIntermediate_ += (N_ + 1) * generalConstraints->getJacobianStateNonZeroCountIntermediate();
        nonZeroJacCountTerminal_ += generalConstraints->getJacobianStateNonZeroCountTerminal();
        nonZeroJacCount_ = nonZeroJacCountIntermediate_ + nonZeroJacCountTerminal_;

        discreteJac_.resize(nonZeroJacCount_);
        discreteIRow_.resize(nonZeroJacCount_);
        discreteJCol_.resize(nonZeroJacCount_);
    }

    virtual VectorXs eval() override
    {
        size_t constraintSize = 0;
        size_t discreteInd = 0;

        for (size_t n = 0; n < N_ + 1; ++n)
        {
            SCALAR tShot = timeGrid_->getShotStartTime(n);
            for (auto constraint : constraints_)
            {
                constraint->setCurrentStateAndControl(
                    w_->getOptimizedState(n), controlSpliner_->evalSpline(tShot, n), tShot);
                constraintSize = constraint->getIntermediateConstraintsCount();
                if (constraintSize > 0)
                {
                    discreteConstraints_.segment(discreteInd, constraintSize) = constraint->evaluateIntermediate();
                    discreteInd += constraintSize;
                }
            }
        }

        for (auto constraint : constraints_)
        {
            constraintSize = constraint->getTerminalConstraintsCount();
            if (constraintSize > 0)
            {
                discreteConstraints_.segment(discreteInd, constraintSize) = constraint->evaluateTerminal();
                discreteInd += constraintSize;
            }
        }

        return discreteConstraints_;
    }

    virtual VectorXs evalSparseJacobian() override
    {
        size_t jacSize = 0;
        size_t discreteInd = 0;

        for (size_t n = 0; n < N_ + 1; ++n)
        {
            SCALAR tShot = timeGrid_->getShotStartTime(n);

            for (auto constraint : constraints_)
            {
                constraint->setCurrentStateAndControl(
                    w_->getOptimizedState(n), controlSpliner_->evalSpline(tShot, n), tShot);
                jacSize = constraint->getJacobianStateNonZeroCountIntermediate();
                if (jacSize > 0)
                {
                    discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianStateSparseIntermediate();
                    discreteInd += jacSize;
                }

                jacSize = constraint->getJacobianInputNonZeroCountIntermediate();
                if (jacSize > 0)
                {
                    discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianInputSparseIntermediate();
                    discreteInd += jacSize;
                }
            }
        }

        for (auto constraint : constraints_)
        {
            jacSize = constraint->getJacobianStateNonZeroCountTerminal();
            if (jacSize > 0)
            {
                discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianStateSparseTerminal();
                discreteInd += jacSize;
            }
            jacSize = constraint->getJacobianInputNonZeroCountTerminal();
            if (jacSize > 0)
            {
                discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianInputSparseTerminal();
                discreteInd += jacSize;
            }
        }

        return discreteJac_;
    }

    virtual size_t getNumNonZerosJacobian() override { return nonZeroJacCount_; }
    virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        size_t discreteInd = 0;
        size_t rowOffset = 0;
        size_t nnEle = 0;

        for (size_t n = 0; n < N_ + 1; ++n)
        {
            for (auto constraint : constraints_)
            {
                nnEle = constraint->getJacobianStateNonZeroCountIntermediate();
                if (nnEle > 0)
                {
                    Eigen::VectorXi iRowStateIntermediate(constraint->getJacobianStateNonZeroCountIntermediate());
                    Eigen::VectorXi jColStateIntermediate(constraint->getJacobianStateNonZeroCountIntermediate());
                    constraint->sparsityPatternStateIntermediate(iRowStateIntermediate, jColStateIntermediate);
                    discreteIRow_.segment(discreteInd, nnEle) = iRowStateIntermediate.array() + rowOffset;
                    discreteJCol_.segment(discreteInd, nnEle) = jColStateIntermediate.array() + w_->getStateIndex(n);
                    discreteInd += nnEle;
                }

                nnEle = constraint->getJacobianInputNonZeroCountIntermediate();
                if (nnEle > 0)
                {
                    Eigen::VectorXi iRowInputIntermediate(constraint->getJacobianInputNonZeroCountIntermediate());
                    Eigen::VectorXi jColInputIntermediate(constraint->getJacobianInputNonZeroCountIntermediate());
                    constraint->sparsityPatternInputIntermediate(iRowInputIntermediate, jColInputIntermediate);
                    discreteIRow_.segment(discreteInd, nnEle) = iRowInputIntermediate.array() + rowOffset;
                    discreteJCol_.segment(discreteInd, nnEle) = jColInputIntermediate.array() + w_->getControlIndex(n);
                    discreteInd += nnEle;
                }

                rowOffset += constraint->getJacobianStateNonZeroCountIntermediate() +
                             constraint->getJacobianInputNonZeroCountIntermediate();
            }
        }

        for (auto constraint : constraints_)
        {
            nnEle = constraint->getJacobianStateNonZeroCountTerminal();
            if (nnEle > 0)
            {
                Eigen::VectorXi iRowStateIntermediate(constraint->getJacobianStateNonZeroCountTerminal());
                Eigen::VectorXi jColStateIntermediate(constraint->getJacobianStateNonZeroCountTerminal());
                constraint->sparsityPatternStateTerminal(iRowStateIntermediate, jColStateIntermediate);
                discreteIRow_.segment(discreteInd, nnEle) = iRowStateIntermediate.array() + rowOffset;
                discreteJCol_.segment(discreteInd, nnEle) = jColStateIntermediate.array() + w_->getStateIndex(N_);
                discreteInd += nnEle;
            }

            nnEle = constraint->getJacobianInputNonZeroCountTerminal();
            if (nnEle > 0)
            {
                Eigen::VectorXi iRowInputIntermediate(constraint->getJacobianInputNonZeroCountTerminal());
                Eigen::VectorXi jColInputIntermediate(constraint->getJacobianInputNonZeroCountTerminal());
                constraint->sparsityPatternInputTerminal(iRowInputIntermediate, jColInputIntermediate);
                discreteIRow_.segment(discreteInd, nnEle) = iRowInputIntermediate.array() + rowOffset;
                discreteJCol_.segment(discreteInd, nnEle) = jColInputIntermediate.array() + w_->getControlIndex(N_);
                discreteInd += nnEle;
            }
            rowOffset +=
                constraint->getJacobianStateNonZeroCountTerminal() + constraint->getJacobianInputNonZeroCountTerminal();
        }

        iRow_vec = discreteIRow_;
        jCol_vec = discreteJCol_;
    }

    virtual VectorXs getLowerBound() override
    {
        size_t discreteInd = 0;
        size_t constraintSize = 0;

        for (size_t n = 0; n < N_ + 1; ++n)
        {
            for (auto constraint : constraints_)
            {
                constraintSize = constraint->getIntermediateConstraintsCount();
                if (constraintSize > 0)
                {
                    discreteLowerBound_.segment(discreteInd, constraintSize) = constraint->getLowerBoundsIntermediate();
                    discreteInd += constraintSize;
                }
            }
        }

        for (auto constraint : constraints_)
        {
            constraintSize = constraint->getTerminalConstraintsCount();
            if (constraintSize > 0)
            {
                discreteLowerBound_.segment(discreteInd, constraintSize) = constraint->getLowerBoundsTerminal();
                discreteInd += constraintSize;
            }
        }

        return discreteLowerBound_;
    }

    virtual VectorXs getUpperBound() override
    {
        size_t discreteInd = 0;
        size_t constraintSize = 0;

        for (size_t n = 0; n < N_ + 1; ++n)
        {
            for (auto constraint : constraints_)
            {
                constraintSize = constraint->getIntermediateConstraintsCount();
                if (constraintSize > 0)
                {
                    discreteUpperBound_.segment(discreteInd, constraintSize) = constraint->getUpperBoundsIntermediate();
                    discreteInd += constraintSize;
                }
            }
        }

        for (auto constraint : constraints_)
        {
            constraintSize = constraint->getTerminalConstraintsCount();
            if (constraintSize > 0)
            {
                discreteUpperBound_.segment(discreteInd, constraintSize) = constraint->getUpperBoundsTerminal();
                discreteInd += constraintSize;
            }
        }

        return discreteUpperBound_;
    }

    virtual size_t getConstraintSize() override { return constraintsCount_; }
private:
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    std::shared_ptr<SplinerBase<control_vector_t, SCALAR>> controlSpliner_;
    std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid_;
    size_t N_;

    std::vector<std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> constraints_;

    size_t constraintsCount_;
    size_t constraintsIntermediateCount_;
    size_t constraintsTerminalCount_;

    VectorXs discreteConstraints_;
    VectorXs discreteLowerBound_;
    VectorXs discreteUpperBound_;

    VectorXs discreteJac_;
    Eigen::VectorXi discreteIRow_;
    Eigen::VectorXi discreteJCol_;

    size_t nonZeroJacCount_;
    size_t nonZeroJacCountIntermediate_;
    size_t nonZeroJacCountTerminal_;
};
}
}
