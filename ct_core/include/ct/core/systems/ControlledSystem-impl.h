/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, bool CONT_T>
ControlledSystem<MANIFOLD, CONT_T>::ControlledSystem(const int control_dim, const SYSTEM_TYPE& type)
    : System<MANIFOLD, CONT_T>(type), controller_(nullptr)
{
    controlAction_.resize(control_dim);
}

template <typename MANIFOLD, bool CONT_T>
ControlledSystem<MANIFOLD, CONT_T>::ControlledSystem(std::shared_ptr<Controller_t> controller, const SYSTEM_TYPE& type)
    : System<MANIFOLD, CONT_T>(type), controller_(controller)
{
    controlAction_.resize(controller->GetControlDim());
    controlAction_.setZero();
}

template <typename MANIFOLD, bool CONT_T>
ControlledSystem<MANIFOLD, CONT_T>::ControlledSystem(const ControlledSystem& arg)
    : System<MANIFOLD, CONT_T>(arg), controlAction_(arg.controlAction_)
{
    if (arg.controller_)
        controller_ = std::shared_ptr<Controller_t>(arg.controller_->clone());
}

template <typename MANIFOLD, bool CONT_T>
void ControlledSystem<MANIFOLD, CONT_T>::setController(const std::shared_ptr<Controller_t>& controller)
{
    if(controller != nullptr) {
      controller_ = controller;
      controlAction_.resize(controller->GetControlDim());
      controlAction_.setZero();
    }
}

template <typename MANIFOLD, bool CONT_T>
void ControlledSystem<MANIFOLD, CONT_T>::getController(std::shared_ptr<Controller_t>& controller) const
{
    controller = controller_;
}

template <typename MANIFOLD, bool CONT_T>
auto ControlledSystem<MANIFOLD, CONT_T>::getController() -> std::shared_ptr<Controller_t>
{
    return controller_;
}

template <typename MANIFOLD, bool CONT_T>
void ControlledSystem<MANIFOLD, CONT_T>::computeDynamics(const MANIFOLD& m,
    const Time_t& t,
    typename MANIFOLD::Tangent& derivative)
{
    if (controller_)
        controller_->computeControl(m, t, controlAction_);
    else
    {
        controlAction_.setZero();
    }

    computeControlledDynamics(m, t, controlAction_, derivative);
}

template <typename MANIFOLD, bool CONT_T>
auto ControlledSystem<MANIFOLD, CONT_T>::getLastControlAction() const -> control_vector_t
{
    return controlAction_;
}

}  // namespace core
}  // namespace ct
