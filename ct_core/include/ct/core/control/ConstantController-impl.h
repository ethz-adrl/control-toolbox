/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, bool CONT_T>
ConstantController<MANIFOLD, CONT_T>::ConstantController(control_vector_t& u) : Base(), u_(u), derivative_u0_(u.size())
{
    derivative_u0_.setIdentity();
}

template <typename MANIFOLD, bool CONT_T>
ConstantController<MANIFOLD, CONT_T>::ConstantController(const ConstantController<MANIFOLD, CONT_T>& other)
    : Base(other), u_(other.u_), derivative_u0_(other.derivative_u0_)
{
}

template <typename MANIFOLD, bool CONT_T>
ConstantController<MANIFOLD, CONT_T>* ConstantController<MANIFOLD, CONT_T>::clone() const
{
    return new ConstantController<MANIFOLD, CONT_T>(*this);
}

template <typename MANIFOLD, bool CONT_T>
void ConstantController<MANIFOLD, CONT_T>::computeControl(const MANIFOLD& state, const Time_t& tn, control_vector_t& u)
{
    u = u_;
}

template <typename MANIFOLD, bool CONT_T>
void ConstantController<MANIFOLD, CONT_T>::setControl(const control_vector_t& u)
{
    u_ = u;
}

template <typename MANIFOLD, bool CONT_T>
auto ConstantController<MANIFOLD, CONT_T>::getControl() const -> const control_vector_t&
{
    return u_;
}

template <typename MANIFOLD, bool CONT_T>
auto ConstantController<MANIFOLD, CONT_T>::getDerivativeU0(const MANIFOLD& state, const Time_t tn)
    -> ControlMatrix<SCALAR>
{
    return derivative_u0_;
}
}  // namespace core
}  // namespace ct
