/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::CostFunctionQuadraticSimple()
{
    x_deviation_.setZero();
    x_nominal_.setZero();
    Q_.setZero();
    Jl_.setIdentity();
    Jr_.setIdentity();
    u_nominal_.setZero();
    R_.setZero();
    x_final_.setZero();
    Q_final_.setZero();
    u_deviation_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::CostFunctionQuadraticSimple(const state_matrix_t& Q,
    const control_matrix_t& R,
    const MANIFOLD& x_nominal,
    const control_vector_t& u_nominal,
    const MANIFOLD& x_final,
    const state_matrix_t& Q_final)
    : x_nominal_(x_nominal), Q_(Q), u_nominal_(u_nominal), R_(R), x_final_(x_final), Q_final_(Q_final)
{
    x_deviation_.setZero();
    u_deviation_.setZero();
    Jl_.setIdentity();
    Jr_.setIdentity();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::~CostFunctionQuadraticSimple()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::CostFunctionQuadraticSimple(const CostFunctionQuadraticSimple& arg)
    : x_deviation_(arg.x_deviation_),
      x_nominal_(arg.x_nominal_),
      Q_(arg.Q_),
      Jl_(arg.Jl_),
      Jr_(arg.Jr_),
      u_deviation_(arg.u_deviation_),
      u_nominal_(arg.u_nominal_),
      R_(arg.R_),
      x_final_(arg.x_final_),
      Q_final_(arg.Q_final_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>* CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::clone() const
{
    return new CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>(*this);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::setCurrentStateAndControl(const MANIFOLD& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    this->x_ = x;
    this->u_ = u;
    this->t_ = t;

    this->u_deviation_ = u - u_nominal_;
    this->x_deviation_ = x_nominal_.rminus(this->x_, Jl_, Jr_);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::evaluateIntermediate() -> SCALAR
{
    // TODO: including Jr_ might not be required here (identical result)
    SCALAR costQ = SCALAR(0.5) * (x_deviation_.transpose() * Jr_ * Q_ * Jr_.transpose() * x_deviation_)(0);
    SCALAR costR = SCALAR(0.5) * (u_deviation_.transpose() * R_ * u_deviation_)(0);
    return costQ + costR;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename MANIFOLD::Tangent CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediate()
{
    return Q_ * Jr_.transpose() * x_deviation_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeIntermediate() -> state_matrix_t
{
    return Jr_ * Q_ * Jr_.transpose();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediate() -> control_vector_t
{
    return R_ * u_deviation_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeIntermediate() -> control_matrix_t
{
    return R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::stateControlDerivativeIntermediate() -> control_state_matrix_t
{
    return control_state_matrix_t::Zero();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::evaluateTerminal() -> SCALAR
{
    // TODO: including Jr_ might not be required here (identical result)
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> Jl, Jr;  // TODO: ignore Jl cleanly
    typename MANIFOLD::Tangent x_deviation_final = x_final_.rminus(this->x_, Jl, Jr);
    return SCALAR(0.5) * (x_deviation_final.transpose() * Jr * Q_final_ * Jr.transpose() * x_deviation_final)(0);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename MANIFOLD::Tangent CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::stateDerivativeTerminal()
{
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> Jl, Jr;  // TODO: ignore Jl cleanly
    typename MANIFOLD::Tangent x_deviation_final = x_final_.rminus(this->x_, Jl, Jr);
    return Q_final_ * Jr.transpose() * x_deviation_final;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeTerminal() -> state_matrix_t
{
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> Jl, Jr;  // TODO: ignore Jl cleanly
    x_final_.rminus(this->x_, Jl, Jr);
    return Jr * Q_final_ * Jr.transpose();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::updateReferenceState(const MANIFOLD& x_ref)
{
    x_nominal_ = x_ref;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadraticSimple<MANIFOLD, CONTROL_DIM>::updateFinalState(const MANIFOLD& x_final)
{
    x_final_ = x_final;
}

}  // namespace optcon
}  // namespace ct
