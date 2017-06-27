/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINT_DISCRETIZER_H_
#define CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINT_DISCRETIZER_H_

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
template <size_t STATE_DIM, size_t CONTROL_DIM>
class ConstraintDiscretizer : public DiscreteConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteConstraintBase BASE;
	typedef ct::core::StateVector<STATE_DIM> state_vector_t;
	typedef ct::core::StateVectorArray<STATE_DIM> state_vector_array_t;

	typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;
	typedef ct::core::ControlVectorArray<CONTROL_DIM> control_vector_array_t;

	typedef ct::core::TimeArray time_array_t;

	/**
	 * @brief      Default constructor
	 */
	ConstraintDiscretizer(){}

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintDiscretizer(){}

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  w             The optimization variables
	 * @param[in]  c_continuous  The continuous constraints
	 * @param[in]  activeInd     A vector defining at which shots the constraint
	 *                           is active
	 */
	ConstraintDiscretizer(
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
		std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
		std::shared_ptr<TimeGrid> timeGrid,
		size_t N
		)
	:
	w_(w),
	controlSpliner_(controlSpliner),
	timeGrid_(timeGrid),
	N_(N),
	constraintsCount_(0),
	constraintsIntermediateCount_(0),
	constraintsTerminalCount_(0),
	nonZeroJacCount_(0),
	nonZeroJacCountIntermediate_(0),
	nonZeroJacCountTerminal_(0)
	{}


	void setStateInputConstraints(std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> stateInputConstraints)
	{
		constraints_.push_back(stateInputConstraints);
		constraintsIntermediateCount_ += (N_ + 1) * stateInputConstraints->getIntermediateConstraintsCount();
		constraintsTerminalCount_ += stateInputConstraints->getTerminalConstraintsCount();
		constraintsCount_ = constraintsIntermediateCount_ + constraintsTerminalCount_;

		discreteConstraints_.resize(constraintsCount_);
		discreteLowerBound_.resize(constraintsCount_);
		discreteUpperBound_.resize(constraintsCount_);

		nonZeroJacCountIntermediate_ += (N_ + 1) * 
			(stateInputConstraints->getJacobianStateNonZeroCountIntermediate() + stateInputConstraints->getJacobianInputNonZeroCountIntermediate());
		nonZeroJacCountTerminal_ += stateInputConstraints->getJacobianStateNonZeroCountTerminal() + stateInputConstraints->getJacobianInputNonZeroCountTerminal();
		nonZeroJacCount_ = nonZeroJacCountIntermediate_ + nonZeroJacCountTerminal_;

		discreteJac_.resize(nonZeroJacCount_);
		discreteIRow_.resize(nonZeroJacCount_);
		discreteJCol_.resize(nonZeroJacCount_);		
	}

	void setPureStateConstraints(std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> pureStateConstraints)
	{
		constraints_.push_back(pureStateConstraints);
		constraintsIntermediateCount_ += (N_ + 1) * pureStateConstraints->getIntermediateConstraintsCount();
		constraintsTerminalCount_ += pureStateConstraints->getTerminalConstraintsCount();
		constraintsCount_ = constraintsIntermediateCount_ + constraintsTerminalCount_;

		discreteConstraints_.resize(constraintsCount_);
		discreteLowerBound_.resize(constraintsCount_);
		discreteUpperBound_.resize(constraintsCount_);

		nonZeroJacCountIntermediate_ += (N_ + 1) * pureStateConstraints->getJacobianStateNonZeroCountIntermediate();
		nonZeroJacCountTerminal_ += pureStateConstraints->getJacobianStateNonZeroCountTerminal();
		nonZeroJacCount_ = nonZeroJacCountIntermediate_ + nonZeroJacCountTerminal_;

		discreteJac_.resize(nonZeroJacCount_);
		discreteIRow_.resize(nonZeroJacCount_);
		discreteJCol_.resize(nonZeroJacCount_);	
	}

	virtual Eigen::VectorXd eval() override
	{
		size_t constraintSize = 0;
		size_t discreteInd = 0;

		for(size_t n = 0; n < N_ + 1; ++n)
		{
			double tShot = timeGrid_->getShotStartTime(n);
			for(auto constraint : constraints_)
			{
				constraint->setCurrentStateAndControl(w_->getOptimizedState(n), controlSpliner_->evalSpline(tShot, n), tShot);
				constraintSize = constraint->getIntermediateConstraintsCount();
				if(constraintSize > 0)
				{
					discreteConstraints_.segment(discreteInd, constraintSize) = constraint->evaluateIntermediate();
					discreteInd += constraintSize;
				}
			}			
		}

		for(auto constraint : constraints_)
		{
			constraintSize = constraint->getTerminalConstraintsCount();
			if(constraintSize > 0)
			{
				discreteConstraints_.segment(discreteInd, constraintSize) = constraint->evaluateTerminal();
				discreteInd += constraintSize;
			}
		}

		return discreteConstraints_;
	}

	virtual Eigen::VectorXd evalSparseJacobian() override
	{
		size_t jacSize = 0;
		size_t discreteInd = 0;

		for(size_t n = 0; n < N_ + 1; ++n)
		{
			double tShot = timeGrid_->getShotStartTime(n);

			for(auto constraint : constraints_)
			{
				constraint->setCurrentStateAndControl(w_->getOptimizedState(n), controlSpliner_->evalSpline(tShot, n), tShot);
				jacSize = constraint->getJacobianStateNonZeroCountIntermediate();
				if(jacSize > 0)
				{
					discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianStateSparseIntermediate();
					discreteInd += jacSize;
				}

				jacSize = constraint->getJacobianInputNonZeroCountIntermediate();
				if(jacSize > 0)
				{
					discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianInputSparseIntermediate();
					discreteInd += jacSize;	
				}
			}
		}

		for(auto constraint : constraints_)
		{
			jacSize = constraint->getJacobianStateNonZeroCountTerminal();
			if(jacSize > 0)
			{
				discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianStateSparseTerminal();
				discreteInd += jacSize;
			}
			jacSize = constraint->getJacobianInputNonZeroCountTerminal();
			if(jacSize > 0)
			{
				discreteJac_.segment(discreteInd, jacSize) = constraint->jacobianInputSparseTerminal();
				discreteInd += jacSize;
			}
		}

		return discreteJac_;		
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return nonZeroJacCount_;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		size_t discreteInd = 0;
		size_t rowOffset = 0;
		size_t nnEle = 0;
	
		for(size_t n = 0; n < N_ + 1; ++n)
		{

			for(auto constraint : constraints_)
			{
				nnEle = constraint->getJacobianStateNonZeroCountIntermediate();
				if(nnEle > 0)
				{
					Eigen::VectorXi iRowStateIntermediate(constraint->getJacobianStateNonZeroCountIntermediate());
					Eigen::VectorXi jColStateIntermediate(constraint->getJacobianStateNonZeroCountIntermediate());
					constraint->sparsityPatternStateIntermediate(iRowStateIntermediate, jColStateIntermediate);
					discreteIRow_.segment(discreteInd, nnEle) = iRowStateIntermediate.array() + rowOffset;
					discreteJCol_.segment(discreteInd, nnEle) = jColStateIntermediate.array() + w_->getStateIndex(n);
					discreteInd += nnEle;
				}

				nnEle = constraint->getJacobianInputNonZeroCountIntermediate();
				if(nnEle > 0)
				{
					Eigen::VectorXi iRowInputIntermediate(constraint->getJacobianInputNonZeroCountIntermediate());
					Eigen::VectorXi jColInputIntermediate(constraint->getJacobianInputNonZeroCountIntermediate()); 
					constraint->sparsityPatternInputIntermediate(iRowInputIntermediate, jColInputIntermediate);
					discreteIRow_.segment(discreteInd, nnEle) = iRowInputIntermediate.array() + rowOffset;
					discreteJCol_.segment(discreteInd, nnEle) = jColInputIntermediate.array() + w_->getControlIndex(n);
					discreteInd += nnEle;
				}

				rowOffset += constraint->getJacobianStateNonZeroCountIntermediate() + constraint->getJacobianInputNonZeroCountIntermediate();
			}
		}

		for(auto constraint : constraints_)
		{
			nnEle = constraint->getJacobianStateNonZeroCountTerminal();
			if(nnEle > 0)
			{
				Eigen::VectorXi iRowStateIntermediate(constraint->getJacobianStateNonZeroCountTerminal());
				Eigen::VectorXi jColStateIntermediate(constraint->getJacobianStateNonZeroCountTerminal());
				constraint->sparsityPatternStateTerminal(iRowStateIntermediate, jColStateIntermediate);
				discreteIRow_.segment(discreteInd, nnEle) = iRowStateIntermediate.array() + rowOffset;
				discreteJCol_.segment(discreteInd, nnEle) = jColStateIntermediate.array() + w_->getStateIndex(N_);
				discreteInd += nnEle;
			}

			nnEle = constraint->getJacobianInputNonZeroCountTerminal();
			if(nnEle > 0)
			{
				Eigen::VectorXi iRowInputIntermediate(constraint->getJacobianInputNonZeroCountTerminal());
				Eigen::VectorXi jColInputIntermediate(constraint->getJacobianInputNonZeroCountTerminal()); 
				constraint->sparsityPatternInputTerminal(iRowInputIntermediate, jColInputIntermediate);
				discreteIRow_.segment(discreteInd, nnEle) = iRowInputIntermediate.array() + rowOffset;
				discreteJCol_.segment(discreteInd, nnEle) = jColInputIntermediate.array() + w_->getControlIndex(N_);
				discreteInd += nnEle;
			}
			rowOffset += constraint->getJacobianStateNonZeroCountTerminal() + constraint->getJacobianInputNonZeroCountTerminal();
		}

		iRow_vec = discreteIRow_;
		jCol_vec = discreteJCol_;
	}

	virtual Eigen::VectorXd getLowerBound() override
	{
		size_t discreteInd = 0;
		size_t constraintSize = 0;

		for(size_t n = 0; n < N_ + 1; ++n)
		{
			for(auto constraint : constraints_)
			{
				constraintSize = constraint->getIntermediateConstraintsCount();
				if(constraintSize > 0)
				{
					discreteLowerBound_.segment(discreteInd, constraintSize) = constraint->getLowerBoundsIntermediate();
					discreteInd += constraintSize;
				}
			}
		}

		for(auto constraint : constraints_)
		{
			constraintSize = constraint->getTerminalConstraintsCount();
			if(constraintSize > 0)
			{
				discreteLowerBound_.segment(discreteInd, constraintSize) = constraint->getLowerBoundsTerminal();
				discreteInd += constraintSize;
			}
		}

		return discreteLowerBound_;
	}

	virtual Eigen::VectorXd getUpperBound() override
	{
		size_t discreteInd = 0;
		size_t constraintSize = 0;

		for(size_t n = 0; n < N_ + 1; ++n)
		{
			for(auto constraint : constraints_)
			{
				constraintSize = constraint->getIntermediateConstraintsCount();
				if(constraintSize > 0)
				{
					discreteUpperBound_.segment(discreteInd, constraintSize) = constraint->getUpperBoundsIntermediate();
					discreteInd += constraintSize;
				}
			}
		}

		for(auto constraint : constraints_)
		{
			constraintSize = constraint->getTerminalConstraintsCount();
			if(constraintSize > 0)
			{
				discreteUpperBound_.segment(discreteInd, constraintSize) = constraint->getUpperBoundsTerminal();
				discreteInd += constraintSize;
			}
		}

		return discreteUpperBound_;
	}

	virtual size_t getConstraintSize() override
	{
		return constraintsCount_;
	}


private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
	std::shared_ptr<TimeGrid> timeGrid_; 
	size_t N_;

	std::vector<std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>>> constraints_;
	
	size_t constraintsCount_;
	size_t constraintsIntermediateCount_;
	size_t constraintsTerminalCount_;

	Eigen::VectorXd discreteConstraints_;
	Eigen::VectorXd discreteLowerBound_;
	Eigen::VectorXd discreteUpperBound_;

	Eigen::VectorXd discreteJac_;
	Eigen::VectorXi discreteIRow_;
	Eigen::VectorXi discreteJCol_;

	size_t nonZeroJacCount_;
	size_t nonZeroJacCountIntermediate_; 
	size_t nonZeroJacCountTerminal_; 
};



}
}


#endif //CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINT_DISCRETIZER_H_
