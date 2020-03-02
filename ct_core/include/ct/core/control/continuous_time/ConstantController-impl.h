/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {


template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::ConstantController()
{
    u_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::ConstantController(ControlVector<CONTROL_DIM, SCALAR>& u) : u_(u)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::ConstantController(
    const ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>& other)
    : Controller<MANIFOLD, CONTROL_DIM, SCALAR>(other), u_(other.u_)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>* ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::clone() const
{
    return new ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>(*this);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::computeControl(const MANIFOLD& state,
    const SCALAR& t,
    ControlVector<CONTROL_DIM, SCALAR>& controlAction)
{
    controlAction = u_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::computeControl(const MANIFOLD& state,
    const int n,
    ControlVector<CONTROL_DIM, SCALAR>& controlAction)
{
    computeControl(state, SCALAR(n), controlAction);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::setControl(const ControlVector<CONTROL_DIM, SCALAR>& u)
{
    u_ = u;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
const ControlVector<CONTROL_DIM, SCALAR>& ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::getControl() const
{
    return u_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ControlMatrix<CONTROL_DIM, SCALAR> ConstantController<MANIFOLD, CONTROL_DIM, SCALAR>::getDerivativeU0(
    const MANIFOLD& state,
    const SCALAR time)
{
    return ControlMatrix<CONTROL_DIM, SCALAR>::Identity();
}
}  // namespace core
}  // namespace ct
