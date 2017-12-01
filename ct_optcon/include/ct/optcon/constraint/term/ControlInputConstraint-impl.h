/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::ControlInputConstraint(
    const core::ControlVector<CONTROL_DIM> uLow,
    const core::ControlVector<CONTROL_DIM> uHigh)
{
    Base::lb_.resize(CONTROL_DIM);
    Base::ub_.resize(CONTROL_DIM);
    // The terminal state constraint is treated as equality constraint, therefore, ub = lb
    Base::lb_ = uLow.template cast<SCALAR>();
    Base::ub_ = uHigh.template cast<SCALAR>();
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
size_t ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
    return CONTROL_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(
    const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return u;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1>
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return u;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM>::Identity();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return 0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
    return CONTROL_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return core::ControlVector<CONTROL_DIM, SCALAR>::Ones();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ControlInputConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInput(VectorXi& rows, VectorXi& cols)
{
    this->genSparseDiagonalIndices(Eigen::Matrix<int, CONTROL_DIM, 1>::Ones(), rows, cols);
}
}
}
