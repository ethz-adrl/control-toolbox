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

#ifndef CT_CORE_CONTROLLEDSYSTEM_H_
#define CT_CORE_CONTROLLEDSYSTEM_H_

#include "../control/Controller.h"
#include "System.h"

#include <ct/core/types/trajectories/StateVectorArray.h>
#include <ct/core/types/trajectories/ControlVectorArray.h>
#include <ct/core/types/trajectories/TimeArray.h>


namespace ct {
namespace core {

//! A general, non-linear dynamic system with a control input
/*!
 * This describes a general, non-linear dynamic system described by an Ordinary Differential Equation (ODE)
 * of the following form
 *
 * \f[
 *  \dot{x} = f(x,u,t)
 * \f]
 *
 * where \f$ x(t) \f$ is the state, \f$ u(t) \f$ the control input and \f$ t \f$ the time.
 *
 * For implementing your own ControlledSystem, derive from this class.
 *
 * We generally assume that the Controller is a state and time dependent function \f$ u = g(x,t) \f$
 * which allows any ControlledSystem to be re-written as a System of the form
 *
 * \f[
 *  \dot{x} = f(x(t),u(x,t),t) = g(x,t)
 * \f]
 *
 * which can be forward propagated in time with an Integrator.
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of input vector
 * @tparam SCALAR scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ControlledSystem : public System<STATE_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef typename std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> Ptr;

	//! default constructor
	/*!
	 * @param type system type
	 */
	ControlledSystem(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
	: 	System<STATE_DIM, SCALAR>(type),
	  	controller_(nullptr),
	  	state_evolution_(nullptr),
	  	control_evolution_(nullptr),
	  	time_evolution_(nullptr)
	  	{};

	//! constructor
	/*!
	 *
	 * @param controller controller
	 * @param type system type
	 */
	ControlledSystem(
			std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR> > controller,
			const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
	: 	System<STATE_DIM, SCALAR>(type),
	  	controller_(controller),
	  	state_evolution_(nullptr),
	  	control_evolution_(nullptr),
	  	time_evolution_(nullptr)
	  	{};

	//! copy constructor
	ControlledSystem(const ControlledSystem& arg):
		System<STATE_DIM, SCALAR>(arg),
		state_evolution_(nullptr),
		control_evolution_(nullptr),
		time_evolution_(nullptr)
	{
		if(arg.controller_)
			controller_ = std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR> > (arg.controller_->clone());
	}

	//! destructor
	virtual ~ControlledSystem() {};

	//! deep copy
	virtual ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override = 0;

	//! set a new controller
	/*!
	 * @param controller new controller
	 */
	void setController(const std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR> >& controller)
	{
		controller_ = controller;
	}

	//! get the controller instance
	/*!
	 * \todo remove this function (duplicate of getController() below)
	 * @param controller controller instance
	 */
	void getController(std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR> >& controller) const
	{
		controller = controller_;
	}

	//! get the controller instace
	/*!
	 * @return controller instance
	 */
	std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR> > getController()
	{
		return controller_;
	}

	//! compute the dynamics of the system
	/*!
	 * Compute the state derivative by evaluating the system dynamics for a given state. This
	 * calls the controller first and then calls computeControlledDynamics() with the current state
	 * and the resulting control signal.
	 *
	 * \note Generally, this function does not need to be overloaded. Better overload computeControlledDynamics().
	 *
	 * @param state current state
	 * @param t current time
	 * @param derivative state derivative
	 */
	virtual void computeDynamics(
			const StateVector<STATE_DIM, SCALAR>& state,
			const Time& t,
			StateVector<STATE_DIM, SCALAR>& derivative) override
	{
		ControlVector<CONTROL_DIM, SCALAR> controlAction;
		if(controller_)
			controller_->computeControl(state, t, controlAction);
		else
			controlAction.setZero();

		computeControlledDynamics(state, t, controlAction, derivative);

		// todo: move to observer! This should not be here
		if(state_evolution_)
			state_evolution_->push_back(state);

		// todo: move to observer! This should not be here
		if(control_evolution_)
			control_evolution_->push_back(controlAction);

		// todo: move to observer! This should not be here
		if(time_evolution_)
			time_evolution_->push_back(t);
	}


	virtual void computeControlledDynamics(
			const StateVector<STATE_DIM, SCALAR>& state,
			const Time& t,
			const ControlVector<CONTROL_DIM, SCALAR>& control,
			StateVector<STATE_DIM, SCALAR>& derivative
	) = 0;


	//! start logging states
	/*!
	 * \todo remove, this should be implemented in an observer!
	 * @param state_ptr pointer to array to log states
	 */
	void startLoggingStates(std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> state_ptr)
	{
		state_evolution_ = state_ptr;
		state_evolution_->clear();
	}

	//! start logging control inputs
	/*!
	 * \todo remove, this should be implemented in an observer!
	 * @param ctrl_ptr pointer to array to log control input
	 */
	void startLoggingControls(std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>> ctrl_ptr)
	{
		control_evolution_ = ctrl_ptr;
		control_evolution_->clear();
	}

	//! start logging times
	/*!
	 * \todo remove, this should be implemented in an observer!
	 * @param time_ptr pointer to array which logs times
	 */
	void startLoggingTimes(std::shared_ptr<ct::core::TimeArray> time_ptr)
	{
		time_evolution_ = time_ptr;
		time_evolution_->clear();
	}

	//! stop logging states
	/*!
	 * \todo remove, this should be implemented in an observer!
	 */
	void stopLoggingStates() {state_evolution_ = nullptr;}

	//! stop logging controls
	/*!
	 * \todo remove, this should be implemented in an observer!
	 */
	void stopLoggingControls() {control_evolution_ = nullptr;}

	//! stop logging times
	/*!
	 * \todo remove, this should be implemented in an observer!
	 */
	void stopLoggingTimes() {time_evolution_ = nullptr;}



protected:
	std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR> > controller_; //!< the controller instance

	std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> state_evolution_; //!< container for logging the state
	std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>> control_evolution_; //!< container for logging the control
	std::shared_ptr<ct::core::TimeArray> time_evolution_;  //!< container for logging the time

};

}
}

#endif /* CT_CORE_CONTROLLEDSYSTEM_H_ */
