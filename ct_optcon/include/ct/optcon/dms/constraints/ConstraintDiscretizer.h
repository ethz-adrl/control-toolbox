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
		// std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> c_continuous,
		// std::vector<size_t> activeInd)
	:
	w_(w),
	controlSpliner_(controlSpliner),
	timeGrid_(timeGrid),
	N_(N),
	constraintCount(0),
	nonZeroJacCount_(0)
	// activeInd_(activeInd)
	{
		// continuousCount_ = c_continuous_->getConstraintCount();
		// constraintsLocal_.resize(continuousCount_);
		// discreteConstraints_.resize(activeInd_.size() * continuousCount_);
		// discreteLowerBound_.resize(activeInd_.size() * continuousCount_);
		// discreteUpperBound_.resize(activeInd_.size() * continuousCount_);

		// nonZeroJacCount_ = c_continuous_->getConstraintJacobianNonZeroCount();
		// jacLocal_.resize(nonZeroJacCount_);
		// iRowLocal_.resize(nonZeroJacCount_);
		// jColLocal_.resize(nonZeroJacCount_);
		// discreteJac_.resize(activeInd_.size() * nonZeroJacCount_);
		// discreteIRow_.resize(activeInd_.size() * nonZeroJacCount_);
		// discreteJCol_.resize(activeInd_.size() * nonZeroJacCount_);
	}


	void setStateInputConstraints(std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> stateInputConstraints)
	{
		stateInputConstraints_ = stateInputConstraints;
		continuousCount_ += stateInputConstraints_->getConstraintsCount();
		constraintsLocal_.resize(continuousCount_);
		discreteConstraints_.resize(N_ * continuousCount_);
		discreteLowerBound_.resize(N_ * continuousCount_);
		discreteUpperBound_.resize(N_ * continuousCount_);

		nonZeroJacCount_ += stateInputConstraints_->getJacNonZeroCount();
		jacLocal_.resize(nonZeroJacCount_);
		iRowLocal_.resize(nonZeroJacCount_);
		jColLocal_.resize(nonZeroJacCount_);
		discreteJac_.resize(N_ * nonZeroJacCount_);
		discreteIRow_.resize(N_ * nonZeroJacCount_);
		discreteJCol_.resize(N_ * nonZeroJacCount_);		
	}

	void setPureStateConstraints(std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> pureStateConstraints)
	{
		pureStateConstraints_ = pureStateConstraints;
		continuousCount_ += pureStateConstraints_->getConstraintsCount();
		constraintsLocal_.resize(continuousCount_);
		discreteConstraints_.resize(N_ * continuousCount_);
		discreteLowerBound_.resize(N_ * continuousCount_);
		discreteUpperBound_.resize(N_ * continuousCount_);

		nonZeroJacCount_ += pureStateConstraints_->getJacNonZeroCount();
		jacLocal_.resize(nonZeroJacCount_);
		iRowLocal_.resize(nonZeroJacCount_);
		jColLocal_.resize(nonZeroJacCount_);
		discreteJac_.resize(N_ * nonZeroJacCount_);
		discreteIRow_.resize(N_ * nonZeroJacCount_);
		discreteJCol_.resize(N_ * nonZeroJacCount_);	
	}

	virtual Eigen::VectorXd eval() override
	{
		// constraintsLocal_.setZero();
		size_t constraintSize = 0;
		size_t discreteInd = 0;
		Eigen::VectorXd evalStateInput;
		Eigen::VectorXd evalPureState;

		for(size_t n = 0; n < N_ + 1; ++n)
		{
			double tShot = timeGrid_->getShotStartTime(n);			
			stateInputConstraints_->setCurrentStateAndControl(w_->getOptimizedState(n), controlSpliner_->evalSpline(tShot, n), tShot);
			pureStateConstraints_->setCurrentStateAndControl(w_->getOptimizedState(n), controlSpliner_->evalSpline(tShot, n), tShot);

			evalStateInput = stateInputConstraints_->evaluateIntermediate();
			constraintSize = stateInputConstraints_->getIntermediateConstraintsCount();
			discreteConstraints_.segment(discreteInd, constraintSize) = evalStateInput;
			discreteInd += constraintSize;

			evalPureState = pureStateConstraints_->evaluateIntermediate();
			constraintSize = pureStateConstraints_->getIntermediateConstraintsCount();
			discreteConstraints_.segment(discreteInd, constraintSize) = evalPureState;
			discreteInd += constraintSize;
		}

		evalStateInput = stateInputConstraints_->evaluateTerminal();
		constraintSize = stateInputConstraints_->getTerminalConstraintsCount();
		discreteConstraints_.segment(discreteInd, constraintSize) = evalStateInput;
		discreteInd += constraintSize;

		evalPureState = pureStateConstraints_->evaluateTerminal();
		constraintSize = pureStateConstraints_->getTerminalConstraintsCount();
		discreteConstraints_.segment(discreteInd, constraintSize) = evalPureState;
		discreteInd += constraintSize;

		return discreteConstraints_;
	}

	virtual Eigen::VectorXd evalSparseJacobian() override
	{
		jacLocal_.setZero();
		size_t jacSize = 0;
		size_t discreteInd = 0;
		Eigen::VectorXd jacStateInput;
		Eigen::VectorXd jacPureState;

		Eigen::VectorXd jacStateInput2;
		Eigen::VectorXd jacPureState2;

		for(size_t n = 0; n < N_; ++n)
		{
			double tShot = timeGrid_->getShotStartTime(ind);
			stateInputConstraints_->setCurrentStateAndControl(w_->getOptimizedState(ind), controlSpliner_->evalSpline(tShot, ind), tShot);
			pureStateConstraints_->setCurrentStateAndControl(w_->getOptimizedState(ind), controlSpliner_->evalSpline(tShot, ind), tShot);

			jacStateInput = stateInputConstraints_->jacobianStateSparseIntermediate();
			jacSize = stateInputConstraints_->getJacobianStateNonZeroCountIntermediate();
			discreteJac_.segment(discreteInd, jacSize) = jacStateInput;
			discreteInd += jacSize;

			jacStateInput2 = stateInputConstraints_->jacobianInputSparseIntermediate();
			jacSize = stateInputConstraints_->getJacobianInputNonZeroCountIntermediate();
			discreteJac_.segment(discreteInd, jacSize) = jacStateInput2;
			discreteInd += jacSize;
			
			jacPureState = pureStateConstraints_->jacobianStateSparseIntermediate();
			jacSize =  pureStateConstraints_->getJacobianStateNonZeroCountIntermediate();
			discreteJac_.segment(discreteInd, jacSize) = jacPureState;
			discreteInd += jacSize;

			jacPureState2 = pureStateConstraints_->jacobianInputSparseIntermediate();
			jacSize =  pureStateConstraints_->getJacobianInputNonZeroCountIntermediate();
			discreteJac_.segment(discreteInd, jacSize) = jacPureState2;
			discreteInd += jacSize;		
		}


		return discreteJac_;		
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return N_ * nonZeroJacCount_;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		size_t discreteInd = 0;
		size_t nnEle = 0;
		size_t i = 0;

		for(auto ind : activeInd_)
		{
			nnEle = c_continuous_->generateSparsityPatternJacobian(iRowLocal_, jColLocal_);
			discreteIRow_.segment(discreteInd, nnEle) = iRowLocal_.array() + i * continuousCount_;
			discreteJCol_.segment(discreteInd, nnEle) = jColLocal_.array() + w_->getStateIndex(ind);
			discreteInd += nnEle;
			i++;
		}
		iRow_vec = discreteIRow_;
		jCol_vec = discreteJCol_;
	}

	virtual Eigen::VectorXd getLowerBound() override
	{
		constraintsLocal_.setZero();
		size_t discreteInd = 0;
		size_t constraintSize = 0;

		for(size_t i = 0; i < N_; ++i)
		{
			c_continuous_->getLowerBound(constraintsLocal_, constraintSize);
			discreteLowerBound_.segment(discreteInd, constraintSize) = constraintsLocal_;
			discreteInd += constraintSize;
		}
		return discreteLowerBound_;
	}

	virtual Eigen::VectorXd getUpperBound() override
	{
		constraintsLocal_.setZero();
		size_t discreteInd = 0;
		size_t constraintSize = 0;

		for(size_t i = 0; i < N_; ++i)
		{
			c_continuous_->getUpperBound(constraintsLocal_, constraintSize);
			discreteUpperBound_.segment(discreteInd, constraintSize) = constraintsLocal_;
			discreteInd += constraintSize;
		}
		return discreteUpperBound_;
	}

	virtual size_t getConstraintSize() override
	{
		size_t discreteCount = 0;
		for(size_t i = 0; i < N_; ++i)
			discreteCount += continuousCount_;

		return discreteCount; 
	}


private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
	std::shared_ptr<TimeGrid> timeGrid_; 
	// std::vector<size_t> activeInd_;
	size_t N_;

	std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> stateInputConstraints_;
	std::shared_ptr<LinearConstraintContainer<STATE_DIM, CONTROL_DIM>> pureStateConstraints_;
	
	size_t continuousCount_;
	Eigen::VectorXd constraintsLocal_;
	Eigen::VectorXd discreteConstraints_;
	Eigen::VectorXd discreteLowerBound_;
	Eigen::VectorXd discreteUpperBound_;

	Eigen::VectorXd jacLocal_;
	Eigen::VectorXd discreteJac_;
	Eigen::VectorXi iRowLocal_;
	Eigen::VectorXi discreteIRow_;
	Eigen::VectorXi jColLocal_;
	Eigen::VectorXi discreteJCol_;

	size_t nonZeroJacCount_;
};



}
}


#endif //CT_OPTCON_DMS_CORE_CONSTRAINTS_CONSTRAINT_DISCRETIZER_H_
