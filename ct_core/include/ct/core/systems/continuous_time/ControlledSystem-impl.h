/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::ControlledSystem(const SYSTEM_TYPE& type)
    : System<MANIFOLD, SCALAR>(type), controller_(nullptr)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::ControlledSystem(
    std::shared_ptr<ct::core::Controller<MANIFOLD, CONTROL_DIM, SCALAR>> controller,
    const SYSTEM_TYPE& type)
    : System<MANIFOLD, SCALAR>(type), controller_(controller)
{
    controlAction_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::ControlledSystem(const ControlledSystem& arg)
    : System<MANIFOLD, SCALAR>(arg), controlAction_(arg.controlAction_)
{
    if (arg.controller_)
        controller_ = std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>>(arg.controller_->clone());
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::~ControlledSystem()
{
}


template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::setController(
    const std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>>& controller)
{
    controller_ = controller;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::getController(
    std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>>& controller) const
{
    controller = controller_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, SCALAR>>
ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::getController()
{
    return controller_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::computeDynamics(const MANIFOLD& state,
    const Time_t& t,
    typename MANIFOLD::Tangent& derivative)
{
    if (controller_)
        controller_->computeControl(state, t, controlAction_);
    else
        controlAction_.setZero();

    computeControlledDynamics(state, t, controlAction_, derivative);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
ControlVector<CONTROL_DIM, SCALAR> ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>::getLastControlAction()
{
    return controlAction_;
}

}  // namespace core
}  // namespace ct
