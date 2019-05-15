/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionQuadraticSimple()
{
    x_nominal_.setZero();
    Q_.setZero();
    u_nominal_.setZero();
    R_.setZero();
    x_final_.setZero();
    Q_final_.setZero();
    x_deviation_.setZero();
    u_deviation_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionQuadraticSimple(const state_matrix_t& Q,
    const control_matrix_t& R,
    const state_vector_t& x_nominal,
    const control_vector_t& u_nominal,
    const state_vector_t& x_final,
    const state_matrix_t& Q_final)
    : x_nominal_(x_nominal), Q_(Q), u_nominal_(u_nominal), R_(R), x_final_(x_final), Q_final_(Q_final)
{
    x_deviation_.setZero();
    u_deviation_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::~CostFunctionQuadraticSimple()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionQuadraticSimple(
    const CostFunctionQuadraticSimple& arg)
    : x_deviation_(arg.x_deviation_),
      x_nominal_(arg.x_nominal_),
      Q_(arg.Q_),
      u_deviation_(arg.u_deviation_),
      u_nominal_(arg.u_nominal_),
      R_(arg.R_),
      x_final_(arg.x_final_),
      Q_final_(arg.Q_final_)
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>*
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    this->x_ = x;
    this->u_ = u;
    this->t_ = t;

    this->x_deviation_ = x - x_nominal_;
    this->u_deviation_ = u - u_nominal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
    SCALAR costQ = SCALAR(0.5) * (x_deviation_.transpose() * Q_ * x_deviation_)(0);
    SCALAR costR = SCALAR(0.5) * (u_deviation_.transpose() * R_ * u_deviation_)(0);
    return costQ + costR;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediate()
{
    return Q_ * x_deviation_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeIntermediate()
{
    return Q_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediate()
{
    return R_ * u_deviation_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeIntermediate()
{
    return R_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeIntermediate()
{
    return control_state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
    state_vector_t x_deviation_final = this->x_ - x_final_;
    return SCALAR(0.5) * x_deviation_final.transpose() * Q_final_ * x_deviation_final;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeTerminal()
{
    state_vector_t x_deviation_final = this->x_ - x_final_;
    return Q_final_ * x_deviation_final;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeTerminal()
{
    return Q_final_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::updateReferenceState(const state_vector_t& x_ref)
{
    x_nominal_ = x_ref;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM, SCALAR>::updateFinalState(const state_vector_t& x_final)
{
    x_final_ = x_final;
}


}  // namespace optcon
}  // namespace ct
