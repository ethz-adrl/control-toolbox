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

#ifndef CT_OPTCON_CONSTRAINT_TERM_CONTROL_INPUT_CONSTRAINT_HPP_
#define CT_OPTCON_CONSTRAINT_TERM_CONTROL_INPUT_CONSTRAINT_HPP_

#include "ConstraintBase.h"
#include <ct/core/types/ControlVector.h>
#include <ct/core/internal/traits/DoubleTrait.h>


namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Class for control input constraint.
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ControlInputConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  uLow   The upper control input bound
	 * @param[in]  uHigh  The lower control input bound
	 */
	ControlInputConstraint(
		const core::ControlVector<CONTROL_DIM> uLow,
		const core::ControlVector<CONTROL_DIM> uHigh)
	{
		Base::lb_.resize(CONTROL_DIM);
		Base::ub_.resize(CONTROL_DIM);
		// The terminal state constraint is treated as equality constraint, therefore, ub = lb
		Base::lb_ = uLow;
		Base::ub_ = uHigh;
	}

	virtual ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	virtual size_t getConstraintSize() const override
	{
		return CONTROL_DIM;
	}

	virtual Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
	{
		return u;
	}

	virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
		const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x, 
		const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
		ct::core::ADCGScalar t) override
	{
		return u;
	}

	virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
	{
		return Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>::Zero();
	}

	virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
	{
		return Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM>::Identity();
	}

	virtual size_t getNumNonZerosJacobianState() const override
	{
		return 0;
	}
	
	virtual size_t getNumNonZerosJacobianInput() const override
	{
		return CONTROL_DIM;
	}

	virtual VectorXs jacobianInputSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override
	{
		return core::ControlVector<CONTROL_DIM, SCALAR>::Ones();
	}

	virtual void sparsityPatternInput(VectorXi& rows, VectorXi& cols) override
	{
		this->genDiagonalIndices(CONTROL_DIM, rows, cols);
	}

};

}
}


#endif //CT_OPTCON_CONTROL_INPUT_CONSTRAINT_HPP_