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

#ifndef CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_IMPL_HPP_
#define CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_IMPL_HPP_

namespace ct {
namespace optcon {
namespace tpl {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ObstacleConstraint(
		std::shared_ptr<ct::core::tpl::Ellipsoid<SCALAR>> obstacle,
		std::function<void (const state_vector_t&, Vector3s&)> getPosition,
		std::function<void (const state_vector_t&, Eigen::Matrix<SCALAR, 3, STATE_DIM>&)> getJacobian)
		:
		obstacle_(obstacle),
		xFun_(getPosition),
		dXFun_(getJacobian)
{
	this->lb_.resize(1);
	this->ub_.resize(1);
	this->lb_(0) = SCALAR(0.0);
	this->ub_(0) = std::numeric_limits<SCALAR>::max();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::~ObstacleConstraint()
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
	return new ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ObstacleConstraint(const ObstacleConstraint& arg):
Base(arg),
obstacle_(arg.obstacle_),
xFun_(arg.xFun_),
dXFun_(arg.dXFun_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
	return 1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	Vector3s xRef;
	xFun_(x, xRef);
	val_(0) = obstacle_->insideEllipsoid(xRef);
	return val_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	Eigen::Vector3d xRef;
	Eigen::Matrix<SCALAR, 3, STATE_DIM> dXRef;
	xFun_(x, xRef);
	dXFun_(x, dXRef);
	VectorXs dist = xRef - obstacle_->getPosition(t);
	jac_ = 2 * dist.transpose() * obstacle_->S() * obstacle_->A().transpose() * obstacle_->A() * obstacle_->S().transpose() * dXRef;
	return jac_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	return Eigen::Matrix<SCALAR, 1, CONTROL_DIM>::Zero();
}


} // namespace tpl
} // namespace optcon
} // namespace core

#endif //CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_IMPL_HPP_
