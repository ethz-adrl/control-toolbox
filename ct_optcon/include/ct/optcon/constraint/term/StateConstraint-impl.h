/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const state_vector_t& xLow,
    const state_vector_t& xHigh)
    : Base(xLow, xHigh)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const VectorXs& lb,
    const VectorXs& ub,
    const Eigen::VectorXi& state_sparsity)
    : Base(lb, ub, state_sparsity)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const StateConstraint& arg) : Base(arg)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::~StateConstraint()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return this->sparsity_J_ * x;
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return this->sparsity_J_.template cast<ct::core::ADCGScalar>() * x;
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return this->sparsity_J_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    MatrixXs jac(this->constrSize_, CONTROL_DIM);
    jac.setZero();
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return this->constrSize_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
    return 0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    VectorXs jac(this->constrSize_);
    jac.setConstant(1.0);
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(VectorXi& rows, VectorXi& cols)
{
    this->sparsityPatternSparseJacobian(this->sparsity_, this->constrSize_, rows, cols);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return VectorXs();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInput(VectorXi& rows, VectorXi& cols)
{
    //do nothing
}

}  // namespace optcon
}  // namespace ct
