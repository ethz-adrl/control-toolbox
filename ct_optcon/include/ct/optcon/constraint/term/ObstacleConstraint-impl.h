/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_IMPL_HPP_
#define CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_IMPL_HPP_

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ObstacleConstraint(
    std::shared_ptr<ct::core::tpl::Ellipsoid<SCALAR>> obstacle,
    std::function<void(const state_vector_t&, Vector3s&)> getPosition,
    std::function<void(const state_vector_t&, Eigen::Matrix<SCALAR, 3, STATE_DIM>&)> getJacobian)
    : obstacle_(obstacle), xFun_(getPosition), dXFun_(getJacobian)
{
    this->lb_.resize(1);
    this->ub_.resize(1);
    this->lb_(0) = SCALAR(0.0);
    this->ub_(0) = std::numeric_limits<SCALAR>::max();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::~ObstacleConstraint()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ObstacleConstraint(const ObstacleConstraint& arg)
    : Base(arg), obstacle_(arg.obstacle_), xFun_(arg.xFun_), dXFun_(arg.dXFun_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
    return 1;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(
    const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    Vector3s xRef;
    xFun_(x, xRef);
    val_(0) = obstacle_->insideEllipsoid(xRef);
    return val_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    Vector3s xRef;
    Eigen::Matrix<SCALAR, 3, STATE_DIM> dXRef;
    xFun_(x, xRef);
    dXFun_(x, dXRef);
    VectorXs dist = xRef - obstacle_->x0();
    jac_ = 2 * dist.transpose() * obstacle_->S() * obstacle_->A().transpose() * obstacle_->A() *
           obstacle_->S().transpose() * dXRef;
    return jac_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ObstacleConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, 1, CONTROL_DIM>::Zero();
}


}  // namespace optcon
}  // namespace core

#endif  //CT_OPTCON_CONSTRAINT_TERM_CONSTRAINT_OBSTACLE_IMPL_HPP_
