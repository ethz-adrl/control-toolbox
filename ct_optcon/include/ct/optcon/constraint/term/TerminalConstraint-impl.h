/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::TerminalConstraint(const core::StateVector<STATE_DIM, SCALAR> xf)
{
    xF_ = xf;
    Base::lb_.resize(STATE_DIM);
    Base::ub_.resize(STATE_DIM);
    // The terminal state constraint is treated as equality constraint, therefore, ub = lb
    Base::lb_.setConstant(SCALAR(0.0));
    Base::ub_.setConstant(SCALAR(0.0));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>* TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::TerminalConstraint(const TerminalConstraint& arg)
    : Base(arg), xF_(arg.xF_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::~TerminalConstraint()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
    return STATE_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return x - xF_;
}

#ifdef CPPADCG
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1>
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return x - xF_.template cast<ct::core::ADCGScalar>();
}
#endif

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>::Identity();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM>::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return STATE_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
    return 0;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparse(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return core::StateVector<STATE_DIM, SCALAR>::Ones();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void TerminalConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(VectorXi& rows, VectorXi& cols)
{
    this->genDiagonalIndices(STATE_DIM, rows, cols);
}
}
}
