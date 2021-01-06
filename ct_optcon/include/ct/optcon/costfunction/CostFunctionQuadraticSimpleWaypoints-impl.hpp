/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::CostFunctionQuadraticSimpleWaypoints(
    const state_matrix_t& Q,
    const control_matrix_t& R,
    const std::vector<MANIFOLD>& x_nominal,
    const std::vector<control_vector_t>& u_nominal,
    const MANIFOLD& x_final,
    const state_matrix_t& Q_final,
    const SCALAR T_final)
    : x_nominal_(x_nominal),
      Q_(Q),
      u_nominal_(u_nominal),
      R_(R),
      x_final_(x_final),
      Q_final_(Q_final),
      T_final_(T_final)
{
    if (x_nominal.size() != u_nominal.size())
        throw std::runtime_error("dimensions in CostFunctionQuadraticSimpleWaypoints do not match.");
    if (x_nominal.size() == 0)
        throw std::runtime_error("you must provide at least one waypoint in CostFunctionQuadraticSimpleWaypoints.");


    x_deviation_.setZero();
    u_deviation_.setZero();
    Adj_.setIdentity();
    Jl_.setIdentity();
    Jr_.setIdentity();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::CostFunctionQuadraticSimpleWaypoints(
    const CostFunctionQuadraticSimpleWaypoints& arg)
    : x_deviation_(arg.x_deviation_),
      x_nominal_(arg.x_nominal_),
      Q_(arg.Q_),
      Adj_(arg.Adj_),
      Jl_(arg.Jl_),
      Jr_(arg.Jr_),
      u_deviation_(arg.u_deviation_),
      u_nominal_(arg.u_nominal_),
      R_(arg.R_),
      x_final_(arg.x_final_),
      Q_final_(arg.Q_final_),
      T_final_(arg.T_final_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>*
CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::clone() const
{
    return new CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>(*this);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::setCurrentStateAndControl(const MANIFOLD& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    this->x_ = x;
    this->u_ = u;
    this->t_ = t;

    // compute which waypoint is active
    SCALAR percent = std::min(t / T_final_, 1.0);
    size_t waypoint_idx = std::min((size_t)(percent * x_nominal_.size()), x_nominal_.size()-1);

    x_deviation_ = this->x_.rminus(this->x_nominal_[waypoint_idx], Jl_, Jr_);  // error expressed w.r.t. x

    Adj_ = Jl_;  //x_deviation_.exp().adj();

    u_deviation_ = u - u_nominal_[waypoint_idx];
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::evaluateIntermediate() -> SCALAR
{
    SCALAR costQ = SCALAR(0.5) * (x_deviation_.transpose() * Q_ * x_deviation_)(0);
    SCALAR costR = SCALAR(0.5) * (u_deviation_.transpose() * R_ * u_deviation_)(0);
    return costQ + costR;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename MANIFOLD::Tangent CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediate()
{
    return Jl_.transpose() * Q_ * x_deviation_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeIntermediate() -> state_matrix_t
{
    return Jl_.transpose() * Q_ * Jl_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediate() -> control_vector_t
{
    return R_ * u_deviation_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::controlSecondDerivativeIntermediate()
    -> control_matrix_t
{
    return R_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::stateControlDerivativeIntermediate()
    -> control_state_matrix_t
{
    return control_state_matrix_t::Zero(CONTROL_DIM, STATE_DIM);  // todo: resize properly
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::evaluateTerminal() -> SCALAR
{
    typename MANIFOLD::Tangent x_deviation_final = this->x_.rminus(this->x_final_);
    return SCALAR(0.5) * (x_deviation_final.transpose() * Q_final_ * x_deviation_final)(0);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
typename MANIFOLD::Tangent CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::stateDerivativeTerminal()
{
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> Jl, Jr;
    typename MANIFOLD::Tangent x_deviation_final = this->x_.rminus(this->x_final_, Jl, Jr);
    return Jl.transpose() * Q_final_ * x_deviation_final;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::stateSecondDerivativeTerminal() -> state_matrix_t
{
    Eigen::Matrix<typename MANIFOLD::Scalar, STATE_DIM, STATE_DIM> Jl, Jr;
    typename MANIFOLD::Tangent x_deviation_final = this->x_.rminus(this->x_final_, Jl, Jr);
    return Jl.transpose() * Q_final_ * Jl;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::updateReferenceState(const MANIFOLD& x_ref)
{
    throw std::runtime_error("CostFunctionQuadraticSimpleWaypoints does not implement updateReferenceState.");
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunctionQuadraticSimpleWaypoints<MANIFOLD, CONTROL_DIM>::updateFinalState(const MANIFOLD& x_final)
{
    throw std::runtime_error("CostFunctionQuadraticSimpleWaypoints does not implement updateFinalState.");
}

}  // namespace optcon
}  // namespace ct
