/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunction<MANIFOLD, CONTROL_DIM>::CostFunction() : t_(0.0), 
t_shift_(0.0)
{
    x_ = MANIFOLD::NeutralElement();
    u_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunction<MANIFOLD, CONTROL_DIM>::~CostFunction()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
CostFunction<MANIFOLD, CONTROL_DIM>::CostFunction(const CostFunction& arg)
    : x_(arg.x_), u_(arg.u_), t_(arg.t_), t_shift_(arg.t_shift_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunction<MANIFOLD, CONTROL_DIM>::setCurrentStateAndControl(const MANIFOLD& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    x_ = x;
    u_ = u;
    t_ = t + t_shift_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunction<MANIFOLD, CONTROL_DIM>::getCurrentStateAndControl(MANIFOLD& x, control_vector_t& u, SCALAR& t) const
{
    x = this->x_;
    u = this->u_;
    t = this->t_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void CostFunction<MANIFOLD, CONTROL_DIM>::shiftTime(const SCALAR t)
{
    t_shift_ = t;
}

}  // namespace optcon
}  // namespace ct
