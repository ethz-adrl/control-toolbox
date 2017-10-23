/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

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
