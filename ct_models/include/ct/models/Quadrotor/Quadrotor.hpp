/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>
#include <memory>

#include <ct/core/core.h>

#include <ct/models/Quadrotor/quadrotor_dynamics/declarations.hpp>
#include <ct/models/Quadrotor/quadrotor_dynamics/QuadrotorDynamics.hpp>


namespace ct {
namespace models {

class Quadrotor : public ct::core::ControlledSystem<quadrotor::nStates, quadrotor::nControls>
{
public:
    Quadrotor(std::shared_ptr<ct::core::Controller<quadrotor::nStates, quadrotor::nControls>> controller = nullptr)
        : ct::core::ControlledSystem<quadrotor::nStates, quadrotor::nControls>(controller)
    {
    }

    Quadrotor(const Quadrotor& arg) : ct::core::ControlledSystem<quadrotor::nStates, quadrotor::nControls>(arg) {}
    virtual Quadrotor* clone() const override { return new Quadrotor(*this); }
    virtual void computeControlledDynamics(const ct::core::StateVector<quadrotor::nStates>& state,
        const ct::core::Time& t,
        const ct::core::ControlVector<quadrotor::nControls>& control,
        ct::core::StateVector<quadrotor::nStates>& derivative) override
    {
        derivative = quadrotor_ode(state, control);
    }
};
}
}
