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

#ifndef CT_CORE_INTEGRATORBASE_H_
#define CT_CORE_INTEGRATORBASE_H_

#include <ct/core/types/trajectories/TimeArray.h>
#include <ct/core/types/trajectories/StateVectorArray.h>
#include <ct/core/systems/System.h>

#include "Observer.h"
#include "EventHandler.h"

namespace ct {
namespace core {

//! Base class for all integrators
/*!
 * Base class for integrators for numerically solving Ordinary Differential Equations (ODEs) of the form
 *
 * \f[
 * \dot{x} = f(x,t)
 * \f]
 *
 * By providing a base class, different integrators can be handled in the same way.
 * Check Integrator for actual implementations
 *
 * @tparam STATE_DIM state dimensionality
 */
template <size_t STATE_DIM>
class IntegratorBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! defines the tolerances for integration
	struct IntegratorSettings
	{
		//! default constructor
		IntegratorSettings() :
			absErrTol(1e-9),
			relErrTol(1e-6)
		{
		}
		double absErrTol; //! absolute tolerance
		double relErrTol; //! relative tolerance
	};

	typedef std::shared_ptr<EventHandler<STATE_DIM>> EventHandlerPtr;
	typedef std::vector<EventHandlerPtr, Eigen::aligned_allocator<EventHandlerPtr>> EventHandlerPtrVector;

	//! constructor
	/*!
	 * Creates a standard integrator
	 *
	 * @param system the system (ODE)
	 * @param eventHandlers (optional) standard vector of event handlers
	 */
	IntegratorBase(
		const std::shared_ptr<System<STATE_DIM> >& system,
		const EventHandlerPtrVector& eventHandlers = EventHandlerPtrVector(0)) :
			observer_(eventHandlers),
			system_(system),
			eventHandlers_(eventHandlers)
	{}

	//! destructor
	virtual ~IntegratorBase() {}

	//! resets the stepper
	virtual void reset() {
		observer_.reset();
	}

	//! Equidistant integration based on number of time steps and step length
	/*!
	 * Integrates n steps forward from the current state recording the state and time trajectory.
	 * For a recording free version, see the function below.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param initialState initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param numSteps number of steps to integrate forward
	 * @param dt step size (fixed also for variable step solvers)
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 */
	virtual void integrate_n_steps(
		StateVector<STATE_DIM>& initialState,
		const Time& startTime,
		size_t numSteps,
		double dt,
		StateVectorArray<STATE_DIM>& stateTrajectory,
		TimeArray& timeTrajectory
	) = 0;

	//! Equidistant integration based on number of time steps and step length
	/*!
	 * Integrates n steps forward from the current state recording the state and time trajectory.
	 * For a recording free version, see the function below.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param numSteps number of steps to integrate forward
	 * @param dt step size (fixed also for variable step solvers)
	 */
	virtual void integrate_n_steps(
		StateVector<STATE_DIM>& state,
		const Time& startTime,
		size_t numSteps,
		double dt
	) = 0;

	//! Equidistant integration based on initial and final time as well as step length
	/*!
	 * Integrates forward from the current state recording the state and time trajectory.
	 * For a recording free version, see the function below.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param dt step size (fixed also for variable step solvers)
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 */
	virtual void integrate_const(
		StateVector<STATE_DIM>& state,
		const Time& startTime,
		const Time& finalTime,
		double dt,
		StateVectorArray<STATE_DIM>& stateTrajectory,
		TimeArray& timeTrajectory
	) = 0;

	//! Equidistant integration based on initial and final time as well as step length
	/*!
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param dt step size (fixed also for variable step solvers)
	 */
	virtual void integrate_const(
		StateVector<STATE_DIM>& state,
		const Time& startTime,
		const Time& finalTime,
		double dt
	) = 0;

	//! integrate forward from an initial to a final time using an adaptive scheme
	/*!
	 * Integrates forward in time from an initial to a final time. If an adaptive stepper is used,
	 * the time step is adjusted according to the tolerances set in the constructor. If a fixed step
	 * integrator is used, this function will do fixed step integration.
	 *
	 * Records state and time evolution. For a recording free version see function below
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 * @param dtInitial step size (initial guess, for fixed step integrators it is fixed)
	 */
	virtual void integrate_adaptive(
		StateVector<STATE_DIM>& state,
		const Time& startTime,
		const Time& finalTime,
		StateVectorArray<STATE_DIM>& stateTrajectory,
		TimeArray& timeTrajectory,
		double dtInitial
	) = 0;

	//! integrate forward from an initial to a final time using an adaptive scheme
	/*!
	 * Integrates forward in time from an initial to a final time. If an adaptive stepper is used,
	 * the time step is adjusted according to the tolerances set in the constructor. If a fixed step
	 * integrator is used, this function will do fixed step integration.
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param startTime start time of the integration
	 * @param finalTime the final time of the integration
	 * @param dtInitial step size (initial guess, for fixed step integrators it is fixed)
	 */
	virtual void integrate_adaptive(
		StateVector<STATE_DIM>& state,
		const Time& startTime,
		const Time& finalTime,
		double dtInitial = 0.01
	) = 0;

	//! Integrate system using a given time trajectory
	/*!
	 * Integrates a system using a given time sequence
	 *
	 * \warning Overrides the initial state
	 *
	 * @param state initial state, contains the final state after integration
	 * @param timeTrajectory sequence of time stamps to perform the integration
	 * @param stateTrajectory the resulting state trajectory corresponding to the times provided
	 * @param dtInitial an initial guess for a time step (fixed for fixed step integrators)
	 */
	virtual void integrate_times(
		StateVector<STATE_DIM>& state,
		const TimeArray& timeTrajectory,
		StateVectorArray<STATE_DIM>& stateTrajectory,
        double dtInitial = 0.01
	) = 0;


protected:

	 void retrieveTrajectoriesFromObserver(StateVectorArray<STATE_DIM>& stateTrajectory, TimeArray& timeTrajectory)
	 {
		 stateTrajectory.swap(observer_.stateTrajectory_);
		 timeTrajectory.swap(observer_.timeTrajectory_);
	 }
	 void retrieveStateVectorArrayFromObserver(StateVectorArray<STATE_DIM>& stateTrajectory)
	 {
		 stateTrajectory.swap(observer_.stateTrajectory_);
	 }

	Observer<STATE_DIM> observer_; //! observer

	std::shared_ptr<System<STATE_DIM> > system_; //! pointer to the system

	EventHandlerPtrVector eventHandlers_; //! vector of event handlers

	IntegratorSettings integratorSettings_;	//! tolerances for integration
};

}
}


#endif /* INTEGRATORBASE_H_ */
