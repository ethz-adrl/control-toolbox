/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::ControlledSystem(const SYSTEM_TYPE& type)
    : System<MANIFOLD, CONT_T>(type), controller_(nullptr)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::ControlledSystem(std::shared_ptr<Controller_t> controller,
    const SYSTEM_TYPE& type)
    : System<MANIFOLD, CONT_T>(type), controller_(controller)
{
    controlAction_.setZero();
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::ControlledSystem(const ControlledSystem& arg)
    : System<MANIFOLD, CONT_T>(arg), controlAction_(arg.controlAction_)
{
    if (arg.controller_)
        controller_ = std::shared_ptr<Controller_t>(arg.controller_->clone());
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::~ControlledSystem()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
void ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::setController(const std::shared_ptr<Controller_t>& controller)
{
    controller_ = controller;
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
void ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::getController(std::shared_ptr<Controller_t>& controller) const
{
    controller = controller_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
auto ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::getController() -> std::shared_ptr<Controller_t>
{
    return controller_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
void ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::computeDynamics(const MANIFOLD& m,
    const Time_t& t,
    typename MANIFOLD::Tangent& derivative)
{
    if (controller_)
        controller_->computeControl(m, t, controlAction_);
    else
        controlAction_.setZero();

    computeControlledDynamics(m, t, controlAction_, derivative);
}

template <typename MANIFOLD, size_t CONTROL_DIM, bool CONT_T>
auto ControlledSystem<MANIFOLD, CONTROL_DIM, CONT_T>::getLastControlAction() const -> control_vector_t
{
    return controlAction_;
}

}  // namespace core
}  // namespace ct
