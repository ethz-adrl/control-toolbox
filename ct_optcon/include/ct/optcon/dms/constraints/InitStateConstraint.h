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

#ifndef CT_OPTCON_DMS_CORE_CONSTRAINTS_INIT_STATE_CONSTRAINT_H_
#define CT_OPTCON_DMS_CORE_CONSTRAINTS_INIT_STATE_CONSTRAINT_H_

#include <ct/optcon/nlp/DiscreteConstraintBase.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      The implementation of the DMS initial state constraint
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class InitStateConstraint : public tpl::DiscreteConstraintBase<SCALAR>
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

	typedef tpl::DiscreteConstraintBase<SCALAR> BASE;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	
	/**
	 * @brief      Default constructor
	 */
	InitStateConstraint(){}

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  x0    The initial state
	 * @param[in]  w     The optimization variables
	 */
	InitStateConstraint(
		const state_vector_t& x0,
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w
		)
	:
		w_(w),
		x_0_(x0)
	{
		lb_.setConstant(SCALAR(0.0));
		ub_.setConstant(SCALAR(0.0));
	}

	/**
	 * @brief      Updates the constraint
	 *
	 * @param[in]  x0    The new initial state
	 */
	void updateConstraint(const state_vector_t& x0) {x_0_ = x0;}

	virtual VectorXs eval() override
	{
		return w_->getOptimizedState(0) - x_0_;
	}

	virtual VectorXs evalSparseJacobian() override
	{
		return state_vector_t::Ones();
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		return (size_t) STATE_DIM;
	}

	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		size_t indexNumber = 0;
		indexNumber += BASE::genDiagonalIndices(w_->getStateIndex(0), STATE_DIM, iRow_vec, jCol_vec, indexNumber);
	}

	virtual VectorXs getLowerBound() override
	{
		return lb_;
	}

	virtual VectorXs getUpperBound() override
	{
		return ub_;
	}

	virtual size_t getConstraintSize() override
	{
		return STATE_DIM;
	}

private:
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
	state_vector_t x_0_;

	//Constraint bounds
	state_vector_t lb_;	// lower bound
	state_vector_t ub_;	// upper bound
};

} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_DMS_CORE_CONSTRAINTS_INIT_STATE_CONSTRAINT_H_

