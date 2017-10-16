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
 * @brief      Class for obstacle constraint.
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     INPUT_DIM  The control dimension
 * @tparam     SCALAR     The Scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ObstacleConstraint : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef typename ct::core::tpl::TraitSelector<SCALAR>::Trait Trait;
	typedef ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR> Base;

	typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;

	ObstacleConstraint(
			std::shared_ptr<ct::core::tpl::Ellipsoid<SCALAR>> obstacle,
			std::function<void (const state_vector_t&, Vector3s&)> getPosition,
			std::function<void (const state_vector_t&, Eigen::Matrix<SCALAR, 3, STATE_DIM>&)> getJacobian);

	virtual ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override;

	virtual ~ObstacleConstraint();

	ObstacleConstraint(const ObstacleConstraint& arg);

	virtual size_t getConstraintSize() const override;

	virtual Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

private:
	std::shared_ptr<ct::core::tpl::Ellipsoid<SCALAR>> obstacle_;

	std::function<void (const core::StateVector<STATE_DIM, SCALAR>&, Vector3s&)> xFun_;
	std::function<void (const core::StateVector<STATE_DIM, SCALAR>&, Eigen::Matrix<SCALAR, 3, STATE_DIM>&)> dXFun_;

	core::StateVector<1, SCALAR> val_;
	Eigen::Matrix<SCALAR, 1, STATE_DIM> jac_;
};

} // namespace optcon
} // namespace ct


