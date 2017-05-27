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

namespace ct {
namespace rbd {


template <size_t NJOINTS>
JointPositionController<NJOINTS>::JointPositionController(
		const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition,
		const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity,
		const std::vector<core::PIDController::parameters_t>& parameters
)
{
	assert(parameters.size() == NJOINTS);

	for (size_t i=0; i<NJOINTS; i++)
	{
		jointControllers_.push_back(core::PIDController(parameters[i], core::PIDController::setpoint_t(desiredPosition(i), desiredVelocity(i))));
	}
}

template <size_t NJOINTS>
JointPositionController<NJOINTS>::JointPositionController(
		const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition,
		const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity,
		const core::PIDController::parameters_t& parameters
)
{
	for (size_t i=0; i<NJOINTS; i++)
	{
		jointControllers_.push_back(core::PIDController(parameters, core::PIDController::setpoint_t(desiredPosition(i), desiredVelocity(i))));
	}
}

template <size_t NJOINTS>
void JointPositionController<NJOINTS>::computeControl(const core::StateVector<STATE_DIM>& state, const core::Time& t, core::ControlVector<NJOINTS>& control)
{
	ct::rbd::JointState<NJOINTS> jstate(state);

	for (size_t i=0; i<NJOINTS; i++)
	{
		control(i) = jointControllers_[i].computeControl(jstate.getPosition(i), jstate.getVelocity(i), t);
	}
}

template <size_t NJOINTS>
void JointPositionController<NJOINTS>::setDesiredPosition(const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition)
{
  for (size_t i=0; i<NJOINTS; i++)
  {
	  jointControllers_[i].setDesiredState(desiredPosition(i));
  };
}

template <size_t NJOINTS>
void JointPositionController<NJOINTS>::setDesiredPosition(double desiredPosition, int jointId)
{
  assert(0 <= jointId && jointId < NJOINTS); // assuming first joint has index 0

  jointControllers_[jointId].setDesiredState(desiredPosition);
}

template <size_t NJOINTS>
void JointPositionController<NJOINTS>::setAllPIDGains(double kp, double ki, double kd)
{
	PIDController::parameters_t parameters;
	parameters.k_p = kp;
	parameters.k_i = ki;
	parameters.k_d = kd;

	for (size_t i=0; i<NJOINTS; i++)
	{
		jointControllers_[i].changeParameters(parameters);
	}
}

template <size_t NJOINTS>
void JointPositionController<NJOINTS>::reset()
{
	for (size_t i=0; i<NJOINTS; i++)
	{
		jointControllers_[i].reset();
	}
}


} // namespace rbd
} // namespace ct
