/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunction() : t_(0.0), t_shift_(0.0)
{
    x_.setZero();
    u_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::~CostFunction()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunction(const CostFunction& arg)
    : x_(arg.x_), u_(arg.u_), t_(arg.t_), t_shift_(arg.t_shift_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    x_ = x;
    u_ = u;
    t_ = t + t_shift_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::getCurrentStateAndControl(Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    SCALAR& t) const
{
    x = this->x_;
    u = this->u_;
    t = this->t_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::shiftTime(const SCALAR t)
{
    t_shift_ = t;
}

}  // namespace optcon
}  // namespace ct
