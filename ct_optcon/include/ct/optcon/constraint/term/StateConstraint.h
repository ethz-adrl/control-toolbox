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

#pragma once

namespace ct {
namespace optcon {


/**
 * @brief      Class for state constraint.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
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
	 * @param[in]  xLow   The lower state bound
	 * @param[in]  xHigh  The upper state bound
	 */
	StateConstraint(
		const state_vector_t& xLow,
		const state_vector_t& xHigh);

	virtual ~StateConstraint();

	virtual StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

	virtual size_t getConstraintSize() const override;

	virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
		const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x, 
		const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
		ct::core::ADCGScalar t) override;

	virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual size_t getNumNonZerosJacobianState() const override;

	virtual size_t getNumNonZerosJacobianInput() const override;

	virtual VectorXs jacobianStateSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual void sparsityPatternState(VectorXi& rows, VectorXi& cols) override;

};

}
}


