/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>
#include <memory>

#include <ct/core/core.h>

#include <ct/models/Quadrotor/quadrotor_dynamics/declarations.hpp>
#include <ct/models/Quadrotor/quadrotor_dynamics/QuadrotorDynamics.hpp>


namespace ct {
namespace models {

class Quadrotor final : public ct::core::ControlledSystem<quadrotor::nStates, quadrotor::nControls>
{
public:
    Quadrotor(std::shared_ptr<ct::core::Controller<quadrotor::nStates, quadrotor::nControls>> controller = nullptr)
        : ct::core::ControlledSystem<quadrotor::nStates, quadrotor::nControls>(controller)
    {
    }

    Quadrotor(const Quadrotor& arg) : ct::core::ControlledSystem<quadrotor::nStates, quadrotor::nControls>(arg) {}
    Quadrotor* clone() const override { return new Quadrotor(*this); }
    void computeControlledDynamics(const ct::core::StateVector<quadrotor::nStates>& state,
        const ct::core::Time& t,
        const ct::core::ControlVector<quadrotor::nControls>& control,
        ct::core::StateVector<quadrotor::nStates>& derivative) override
    {
        derivative = quadrotor_ode(state, control);
    }
};
}
}
