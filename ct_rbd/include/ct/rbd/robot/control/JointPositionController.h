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

#ifndef JOINTPOSITIONCONTROLLER_H_
#define JOINTPOSITIONCONTROLLER_H_

#include <vector>
#include <ct/core/control/siso/PIDController.h>


namespace ct {
namespace rbd {

/**
 * \brief A joint position controller using a PID controller for all joints
 *
 * \tparam NJOINTS number of joints of the robot
 */
template <size_t NJOINTS>
class JointPositionController : public ct::core::Controller<2*NJOINTS, NJOINTS>
{
public:
  static const size_t STATE_DIM = 2*NJOINTS;
  static const size_t CONTROL_DIM = NJOINTS;

  typedef std::shared_ptr<JointPositionController<NJOINTS> > Ptr;
  typedef ct::core::PIDController PIDController;

  virtual JointPositionController<NJOINTS>* clone() const override { throw std::runtime_error("Not implemented"); };

	JointPositionController(
			const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition = Eigen::Matrix<double, NJOINTS, 1>::Zero(),
			const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity = Eigen::Matrix<double, NJOINTS, 1>::Zero(),
			const std::vector<PIDController::parameters_t>& parameters = std::vector<PIDController::parameters_t>(NJOINTS, PIDController::parameters_t())
	);
	JointPositionController(
			const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition,
			const Eigen::Matrix<double, NJOINTS, 1>& desiredVelocity,
			const PIDController::parameters_t& parameters
	);
	~JointPositionController() {};

	void computeControl(const core::StateVector<STATE_DIM>& state, const core::Time& t, core::ControlVector<NJOINTS>& control) override;

	void setDesiredPosition(const Eigen::Matrix<double, NJOINTS, 1>& desiredPosition);
	void setDesiredPosition(double desiredPosition, int jointId);

	void setAllPIDGains(double kp, double ki, double kd);

	void reset();

protected:

	std::vector<PIDController> jointControllers_;

};

} // namespace rbd
} // namespace ct

#include "implementation/JointPositionController.h"


#endif /* JOINTPOSITIONCONTROLLER_H_ */
