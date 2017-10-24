/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::StateConstraint(const state_vector_t& xLow,
    const state_vector_t& xHigh)
{
    Base::lb_.resize(STATE_DIM);
    Base::ub_.resize(STATE_DIM);
    // The terminal state constraint is treated as equality constraint, therefore, ub = lb
    Base::lb_ = xLow;
    Base::ub_ = xHigh;
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
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
    return STATE_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateCppadCg(
    const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
    const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
    ct::core::ADCGScalar t)
{
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>::Identity();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR t)
{
    return Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM>::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
    return STATE_DIM;
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
    return state_vector_t::Ones();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternState(VectorXi& rows, VectorXi& cols)
{
    this->genDiagonalIndices(STATE_DIM, rows, cols);
}

}  // namespace optcon
}  // namespace ct
