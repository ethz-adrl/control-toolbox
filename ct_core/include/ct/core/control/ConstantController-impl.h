/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::ConstantController()
{
    u_.setZero();
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::ConstantController(control_vector_t& u) : Base(), u_(u)
{
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::ConstantController(const ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>& other)
    : Base(other), u_(other.u_)
{
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>* ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::clone() const
{
    return new ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>(*this);
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
void ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::computeControl(const MANIFOLD& state,
    const Time_t& tn,
    control_vector_t& u)
{
    u = u_;
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
void ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::setControl(const control_vector_t& u)
{
    u_ = u;
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
auto ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::getControl() const -> const control_vector_t& 
{
    return u_;
}

template <typename MANIFOLD, int CONTROL_DIM, bool CONT_T>
auto ConstantController<MANIFOLD, CONTROL_DIM, CONT_T>::getDerivativeU0(const MANIFOLD& state,
    const Time_t tn) -> ControlMatrix<CONTROL_DIM, SCALAR> 
{
    throw std::runtime_error("getDerivativeU0 not implemented for ConstantController.");
    //return ControlMatrix<CONTROL_DIM, SCALAR>::Identity();
    return ControlMatrix<CONTROL_DIM, SCALAR>();
}
}  // namespace core
}  // namespace ct
