/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::DisturbedSystem()
    : ct::core::ControlledSystem<AUGMENTED_DIM, CONTROL_DIM, SCALAR>(ct::core::SYSTEM_TYPE::GENERAL)
{
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::DisturbedSystem(
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller)
    : ct::core::ControlledSystem<AUGMENTED_DIM, CONTROL_DIM, SCALAR>(
          std::shared_ptr<DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>>(
              new DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>(controller)),
          ct::core::SYSTEM_TYPE::GENERAL)
{
    this->controlAction_.setZero();
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
void DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::setController(
    const std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller)
{
    std::dynamic_pointer_cast<DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>>(this->controller_)
        ->setController(controller);
}

}  // namespace optcon
}  // namespace ct
