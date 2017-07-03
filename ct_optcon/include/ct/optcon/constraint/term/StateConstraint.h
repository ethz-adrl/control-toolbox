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

#ifndef CT_OPTCON_CONSTRAINT_TERM_STATE_CONSTRAINT_HPP_
#define CT_OPTCON_CONSTRAINT_TERM_STATE_CONSTRAINT_HPP_

#include "ConstraintBase.h"
#include <ct/core/types/StateVector.h>

namespace ct {
namespace optcon {
namespace tpl {


/**
 * @brief      Class for state constraint.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class StateConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  xLow   The lower state bound
	 * @param[in]  xHigh  The upper state bound
	 */
	StateConstraint(
		const core::StateVector<STATE_DIM>& xLow,
		const core::StateVector<STATE_DIM>& xHigh)
	{
		Base::lb_.resize(STATE_DIM);
		Base::ub_.resize(STATE_DIM);
		// The terminal state constraint is treated as equality constraint, therefore, ub = lb
		Base::lb_ = xLow;
		Base::ub_ = xHigh;
	}

	virtual StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	virtual size_t getConstraintSize() const override
	{
		return STATE_DIM;
	}

	virtual Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x, const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u, const SCALAR t) override
	{
		return x;
	}

	virtual Eigen::MatrixXd jacobianState(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override
	{
		return Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
	}

	virtual Eigen::MatrixXd jacobianInput(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override
	{
		return Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>::Zero();
	}

	virtual size_t getNumNonZerosJacobianState() const override
	{
		return STATE_DIM;
	}
	virtual size_t getNumNonZerosJacobianInput() const override
	{
		return 0;
	}

	virtual Eigen::VectorXd jacobianStateSparse(const Eigen::Matrix<double, STATE_DIM, 1> &x, const Eigen::Matrix<double, CONTROL_DIM, 1> &u, const double t) override
	{
		return core::StateVector<STATE_DIM>::Ones();
	}

	virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols) override
	{
		this->genDiagonalIndices(STATE_DIM, rows, cols);
	}

};

}

template<size_t STATE_DIM, size_t INPUT_DIM>
using StateConstraint = tpl::StateConstraint<STATE_DIM, INPUT_DIM, double>;

}
}


#endif //CT_OPTCON_CONTROL_INPUT_CONSTRAINT_HPP_