/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ControlInputConstraint(const control_vector_t& lb,
    const control_vector_t& ub)
    : Base(lb, ub)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ControlInputConstraint(const VectorXs& lb,
    const VectorXs& ub,
    const Eigen::VectorXi& control_sparsity)
    : Base(lb, ub, control_sparsity)
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ControlInputConstraint(const ControlInputConstraint& arg)
    : Base(arg)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::~ControlInputConstraint()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::clone()
    const
{
    return new ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(
    const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return this->sparsity_J_ * u;
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return this->sparsity_J_.template cast<ct::core::ADCGScalar>() * u;
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    MatrixXs jac(this->constrSize_, STATE_DIM);
    jac.setZero();
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return this->sparsity_J_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return 0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
    return this->constrSize_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return VectorXs();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(VectorXi& rows, VectorXi& cols)
{
    //    do nothing
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    VectorXs jac(this->constrSize_);
    jac.setConstant(1.0);
    return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInput(VectorXi& rows, VectorXi& cols)
{
    this->sparsityPatternSparseJacobian(this->sparsity_, this->constrSize_, rows, cols);
}

}  // namespace optcon
}  // namespace ct
