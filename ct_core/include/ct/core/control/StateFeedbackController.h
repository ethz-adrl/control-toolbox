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

#ifndef CT_CORE_CONTROL_STATEFEEDBACKCONTROLLER_H_
#define CT_CORE_CONTROL_STATEFEEDBACKCONTROLLER_H_

#include <iostream>

#include "Controller.h"

#include <ct/core/types/trajectories/FeedbackArray.h>
#include <ct/core/types/trajectories/FeedbackTrajectory.h>
#include <ct/core/types/trajectories/ControlTrajectory.h>
#include <ct/core/types/trajectories/StateTrajectory.h>

namespace ct {
namespace core {

//! A linear state feedback controller
/*!
 * A general, discrete, time-varying linear state feedback controller with feedforward action of type
 *
 * \f[
 *
 * u(x,t) = u_{ff}(t) + K(t) x
 *
 * \f]
 *
 * where \f$ u_{ff} \f$ is a time varying feedforward and \f$ K(t) \f$ a time-varying
 * linear feedback controller.
 *
 * @tparam STATE_DIM state vector size
 * @tparam CONTROL_DIM control vector size
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateFeedbackController : public Controller<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! The base (interface) class
	typedef Controller<STATE_DIM, CONTROL_DIM, SCALAR> ControllerBase;

	//! default constructor
	StateFeedbackController(){}

	//! constructor
	/*!
	 * Constructs a state feedback controller. The feedforward and feedback parts
	 * get interpolated where required.
	 * @param uff feedforward controller.
	 * @param K feedback controller.
	 * @param deltaT discretization step
	 * @param t0 initial time
	 * @param intType interpolation type
	 */
	StateFeedbackController(
			const ControlVectorArray<CONTROL_DIM, SCALAR>& uff,
			const FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K,
			const SCALAR& deltaT,
			const SCALAR& t0 = 0.0,
			const InterpolationType& intType = ZOH
	) :
		uff_(uff, deltaT, t0, intType),
		K_(K, deltaT, t0, intType)
	{}

	//! copy constructor
	StateFeedbackController(const StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other) :
		uff_(other.uff_),
		K_(other.K_)
	{}

	//! destructor
	virtual ~StateFeedbackController() {}

	//! deep cloning, required by ControlledSystem
	StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override {
		return new StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	//! computes the control action
	/*!
	 * evaluates the controller using interpolation where required using Interpolation
	 * @param state current state
	 * @param t current time
	 * @param controlAction resulting control action
	 */
	virtual void computeControl(
			const StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& t,
			ControlVector<CONTROL_DIM, SCALAR>& controlAction) override {

		controlAction = uff_.eval(t) + K_.eval(t) * state;
	}

	//! updates the controller
	/*!
	 * sets a new feedforward and feedback controller
	 * @param uff feedforward control action
	 * @param K feedback controller
	 * @param times discretization times
	 */
	void update(
			const DiscreteArray<ControlVector<CONTROL_DIM, SCALAR>>& uff,
			const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>& K,
			const tpl::TimeArray<SCALAR>& times) {
		uff_.setData(uff);
		uff_.setTime(times);
		K_.setData(K);
		K_.setTime(times);
	}

	//! get feedforward array (without timings)
	const DiscreteArray<ControlVector<CONTROL_DIM, SCALAR>>& uff() const { return uff_.getDataArray(); }

	//! get feedback array (without timings
	const DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>> & K() const { return K_.getDataArray(); }

	//! get time array
	const tpl::TimeArray<SCALAR>& time() const {return uff_.getTimeArray();}

	//! get a reference to the feedforward trajectory
	ControlTrajectory<CONTROL_DIM, SCALAR>& getFeedforwardTrajectory() { return uff_; }

	//! get a reference to the feedforward trajectory
	const ControlTrajectory<CONTROL_DIM, SCALAR>& getFeedforwardTrajectory() const { return uff_; }

	//! get a reference to the feedback trajectory
	FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& getFeedbackTrajectory() { return K_;}

	//! get a reference to the feedback trajectory
	const FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& getFeedbackTrajectory() const { return K_;}

	//!  extracts a physically meaningful control trajectory from the given state-feedback law and a reference state trajectory
	void extractControlTrajectory(const StateTrajectory<STATE_DIM, SCALAR>& x_traj, ControlTrajectory<CONTROL_DIM, SCALAR>& u_traj){
		u_traj.clear();

		for(size_t i = 0; i<x_traj.size()-1; i++){
			u_traj.push_back(uff_[i]+K_[i]*x_traj[i], x_traj.getTimeFromIndex(i), true);
		}
	}

private:
	ControlTrajectory<CONTROL_DIM, SCALAR> uff_; //! feedforward control trajectory
	FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR> K_; //! feedback control trajectory

};

} // core
} // ct

#endif /* INCLUDE_CT_CORE_CONTROL_STATEFEEDBACKCONTROLLER_H_ */
