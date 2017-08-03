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


#ifndef CT_CORE_INTEGRATOR_H_
#define CT_CORE_INTEGRATOR_H_

#include <type_traits>
#include <functional>
#include <cmath>

#include "eigenIntegration.h"

#include "IntegratorBase.h"

#include "internal/steppers.h"
#include "internal/stepper2.h"
#include "internal/SteppersODEInt.h"
#include "internal/SteppersCT.h"

namespace ct {
namespace core {

enum IntegrationType
{
    EULER,
    EULERCT,
    RK4,
    RK4CT,
    MODIFIED_MIDPOINT,
    ODE45,
    RK5VARIABLE,
    RK78,
    BULIRSCHSTOER
};


//! Standard Integrator
/*!
 * A standard Integrator for numerically solving Ordinary Differential Equations (ODEs) of the form
 *
 * \f[
 * \dot{x} = f(x,t)
 * \f]
 *
 * \warning This is mostly an interface definition. To create actual integrators use on of the following typedefs
 * which are all templated on the state dimension:
 * - @ref IntegratorEuler
 * - @ref IntegratorModifiedMidpoint
 * - @ref IntegratorRK4
 * - @ref IntegratorRK5Variable
 * - @ref ODE45
 * - @ref IntegratorRK78
 * - @ref IntegratorBulirschStoer
 *
 * Unit test \ref IntegrationTest.cpp illustrates the use of Integrator.h
 *
 *
 * @tparam STATE_DIM the size of the state vector
 * @tparam STEPPER the stepper type
 */
template <size_t STATE_DIM, typename SCALAR = double>
class Integrator : public IntegratorBase<STATE_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! Base (interface) class
	typedef IntegratorBase<STATE_DIM, SCALAR> Base;
	//! constructor
	/*!
	 * Creates a standard integrator
	 *
	 * @param system the system (ODE)
	 * @param eventHandlers optional event handler
	 * @param absErrTol optional absolute error tolerance (for variable step solvers)
	 * @param relErrTol optional relative error tolerance (for variable step solvers)
	 */
	Integrator(
			const std::shared_ptr<System<STATE_DIM, SCALAR> >& system,
			const IntegrationType& intType = IntegrationType::EULERCT,
			const typename Base::EventHandlerPtrVector& eventHandlers = typename Base::EventHandlerPtrVector(0),
			const SCALAR& absErrTol = SCALAR(1e-9),
			const SCALAR& relErrTol = SCALAR(1e-6)
	) :
		Base(system, eventHandlers)
	{
		Base::integratorSettings_.absErrTol = absErrTol;
		Base::integratorSettings_.relErrTol = relErrTol;

		switch(intType)
		{
			case EULER:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperODEInt<internal::euler_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperODEInt<internal::euler_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
				break;
			}

			case EULERCT:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperEulerCT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperEulerCT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
				break;
			}

			case RK4:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperODEInt<internal::runge_kutta_4_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperODEInt<internal::runge_kutta_4_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
				break;
			}

			case RK4CT:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperRK4CT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperRK4CT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
				break;
			}
			case MODIFIED_MIDPOINT:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperODEInt<internal::modified_midpoint_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperODEInt<internal::modified_midpoint_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
				break;
			}
			case ODE45:
			{
				break;
			}
			case RK5VARIABLE:
			{
				break;
			}
			case RK78:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperODEInt<internal::runge_kutta_fehlberg78_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperODEInt<internal::runge_kutta_fehlberg78_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());

				break;
			}
			case BULIRSCHSTOER:
			{
				integratorStepper_ = std::shared_ptr<internal::StepperODEInt<internal::bulirsch_stoer_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
					new internal::StepperODEInt<internal::bulirsch_stoer_t<STATE_DIM, SCALAR>, Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());

				break;
			}
			default:
			{
				throw std::runtime_error("Unknown integration type");
				break;
			}			    
		}

		setupSystem();
	}

	//! resets the stepper
	void reset() override
	{
		Base::reset();
	}

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
	 * @param stateTrajectory state evolution over time
	 * @param timeTrajectory time trajectory corresponding to state trajectory
	 */
	virtual void integrate_n_steps(
			StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& startTime,
			size_t numSteps,
			SCALAR dt,
			StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
			tpl::TimeArray<SCALAR>& timeTrajectory) override 
	{
		reset();

		integratorStepper_->integrate_n_steps(Base::observer_.observeWrap, systemFunction_, state, startTime, numSteps, dt);

		Base::retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);	
	}

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
			StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& startTime,
			size_t numSteps,
			SCALAR dt) override 
	{

		reset();
		integratorStepper_->integrate_n_steps(systemFunction_, state, startTime, numSteps, dt);
		
		// boost::numeric::odeint::integrate_n_steps(stepper_, );
	}

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
			StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& startTime,
			const SCALAR& finalTime,
			SCALAR dt,
			StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
			tpl::TimeArray<SCALAR>& timeTrajectory
	) override {

		reset();

		integratorStepper_->integrate_const(Base::observer_.observeWrap, systemFunction_, state, startTime, finalTime, dt);
		// integrate_const(state, startTime, finalTime, dt);

		Base::retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
	}

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
			StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& startTime,
			const SCALAR& finalTime,
			SCALAR dt
	) override {

		reset();

		integratorStepper_->integrate_const(systemFunction_, state, startTime, finalTime, dt);
	}

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
	void integrate_adaptive(
			StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& startTime,
			const SCALAR& finalTime,
			StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
			tpl::TimeArray<SCALAR>& timeTrajectory,
			const SCALAR dtInitial = SCALAR(0.01)) override 
	{

		reset();

		integratorStepper_->integrate_adaptive(Base::observer_.observeWrap, systemFunction_, state, startTime, finalTime, dtInitial);

		// integrate_adaptive_specialized<Stepper>(state, startTime, finalTime, dtInitial);

		Base::retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);

		state = stateTrajectory.back();
	}

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
	void integrate_adaptive(
			StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& startTime,
			const SCALAR& finalTime,
			SCALAR dtInitial = SCALAR(0.01)
	) override {

		reset();

		integratorStepper_->integrate_adaptive(Base::observer_.observeWrap, systemFunction_, state, startTime, finalTime, dtInitial);

		// integrate_adaptive_specialized<Stepper>(state, startTime, finalTime, dtInitial);
	}

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
	void integrate_times(
			StateVector<STATE_DIM, SCALAR>& state,
			const tpl::TimeArray<SCALAR>& timeTrajectory,
			StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
			SCALAR dtInitial = SCALAR(0.01)
	) override {

		reset();

		// integrate_times_specialized<Stepper>(state, timeTrajectory, dtInitial);
		integratorStepper_->integrate_times(Base::observer_.observeWrap, systemFunction_, state, timeTrajectory, dtInitial);

		Base::retrieveStateVectorArrayFromObserver(stateTrajectory);
	}

private:

	//! sets up the lambda function
	void setupSystem() {

		systemFunction_ = [this]( const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x, Eigen::Matrix<SCALAR, STATE_DIM, 1>& dxdt, SCALAR t) {
			const StateVector<STATE_DIM, SCALAR>& xState(static_cast<const StateVector<STATE_DIM, SCALAR>& >(x));
			StateVector<STATE_DIM, SCALAR>& dxdtState(static_cast<StateVector<STATE_DIM, SCALAR>& >(dxdt));
			this->system_->computeDynamics(xState, t, dxdtState);
		};

		reset();
	}

	// template <typename S = Stepper>
	// typename std::enable_if<std::is_same<S, internal::runge_kutta_dopri5_t<STATE_DIM, SCALAR>>::value, void>::type
	// initializeStepper(const StateVector<STATE_DIM, SCALAR>& initialState, const SCALAR& t, SCALAR dt) {

	// 	/**do nothing, runge_kutta_5_t does not have a init method */
	// }


	// template <typename S = Stepper>
	// typename std::enable_if<std::is_same<S, internal::dense_runge_kutta5_t<STATE_DIM, SCALAR>>::value, void>::type
	// initializeStepper(const StateVector<STATE_DIM, SCALAR>& initialState, const SCALAR& t, SCALAR dt) {

	// 	/** dense steppers runge kutta requires initialization */
	// 	stepper_.initialize(initialState, t, dt);
	// }


	// template <typename S>
	// typename std::enable_if<std::is_same<S, internal::runge_kutta_dopri5_t<STATE_DIM, SCALAR>>::value, void>::type
	// integrate_adaptive_specialized(
	// 		StateVector<STATE_DIM, SCALAR>& initialState,
	// 		const SCALAR& startTime,
	// 		const SCALAR& finalTime,
	// 		SCALAR dtInitial) {

	// 	SCALAR startTime_temp = startTime;

	// 	initializeStepper(initialState, startTime_temp, dtInitial);

	// 	boost::numeric::odeint::integrate_adaptive(
	// 			boost::numeric::odeint::make_controlled<S>(Base::integratorSettings_.absErrTol, Base::integratorSettings_.relErrTol),
	// 			systemFunction_, initialState, startTime, finalTime, dtInitial, Base::observer_.observeWrap);
	// }


	// template <typename S>
	// typename std::enable_if<std::is_same<S, internal::dense_runge_kutta5_t<STATE_DIM, SCALAR>>::value, void>::type
	// integrate_adaptive_specialized(
	// 		StateVector<STATE_DIM, SCALAR>& initialState,
	// 		const SCALAR& startTime,
	// 		const SCALAR& finalTime,
	// 		SCALAR dtInitial) {

	// 	SCALAR startTime_temp = startTime;

	// 	initializeStepper(initialState, startTime_temp, dtInitial);

	// 	boost::numeric::odeint::integrate_adaptive(
	// 			boost::numeric::odeint::make_dense_output(
	// 					Base::integratorSettings_.absErrTol, Base::integratorSettings_.relErrTol, internal::runge_kutta_dopri5_t<STATE_DIM, SCALAR>()),
	// 					systemFunction_, initialState, startTime, finalTime, dtInitial, Base::observer_.observeWrap);
	// }


	// template <typename S>
	// typename std::enable_if<(
	// 		std::is_same<S, internal::euler_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::runge_kutta_4_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::modified_midpoint_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::runge_kutta_fehlberg78_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::bulirsch_stoer_t<STATE_DIM, SCALAR>>::value
	// ), void>::type
	// integrate_adaptive_specialized(
	// 		StateVector<STATE_DIM, SCALAR>& initialState,
	// 		const SCALAR& startTime,
	// 		const SCALAR& finalTime,
	// 		SCALAR dtInitial) {

	// 	boost::numeric::odeint::integrate_adaptive(stepper_, systemFunction_, initialState, startTime, finalTime, dtInitial, Base::observer_.observeWrap);
	// }


	// template <typename S>
	// typename std::enable_if<std::is_same<S, internal::runge_kutta_dopri5_t<STATE_DIM, SCALAR>>::value, void>::type
	// integrate_times_specialized(
	// 		StateVector<STATE_DIM, SCALAR>& initialState,
	// 		const tpl::TimeArray<SCALAR>& timeTrajectory,
	// 		SCALAR dtInitial = SCALAR(0.01)) {

	// 	SCALAR startTime_temp = timeTrajectory.front();

	// 	initializeStepper(initialState, startTime_temp, dtInitial);

	// 	boost::numeric::odeint::integrate_times(
	// 			boost::numeric::odeint::make_controlled<S>(Base::integratorSettings_.absErrTol, Base::integratorSettings_.relErrTol),
	// 			systemFunction_, initialState, &timeTrajectory.front(), &timeTrajectory.back()+1, dtInitial, Base::observer_.observeWrap);
	// }


	// template <typename S>
	// typename std::enable_if<std::is_same<S, internal::dense_runge_kutta5_t<STATE_DIM, SCALAR>>::value, void>::type
	// integrate_times_specialized(
	// 		StateVector<STATE_DIM, SCALAR>& initialState,
	// 		const tpl::TimeArray<SCALAR>& timeTrajectory,
	// 		SCALAR dtInitial = SCALAR(0.01)) {

	// 	StateVector<STATE_DIM, SCALAR> initialStateInternal = initialState;
	// 	SCALAR startTime_temp = timeTrajectory.front();

	// 	initializeStepper(initialStateInternal, startTime_temp, dtInitial);

	// 	boost::numeric::odeint::integrate_times(
	// 			boost::numeric::odeint::make_dense_output(
	// 					Base::integratorSettings_.absErrTol, Base::integratorSettings_.relErrTol, internal::runge_kutta_dopri5_t<STATE_DIM, SCALAR>()),
	// 					systemFunction_, initialStateInternal, &timeTrajectory.front(), &timeTrajectory.back()+1, dtInitial, Base::observer_.observeWrap);
	// }


	// template <typename S>
	// typename std::enable_if<(
	// 		std::is_same<S, internal::euler_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::runge_kutta_4_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::modified_midpoint_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::runge_kutta_fehlberg78_t<STATE_DIM, SCALAR>>::value ||
	// 		std::is_same<S, internal::bulirsch_stoer_t<STATE_DIM, SCALAR>>::value
	// ), void>::type
	// integrate_times_specialized(
	// 		StateVector<STATE_DIM, SCALAR>& initialState,
	// 		const tpl::TimeArray<SCALAR>& timeTrajectory,
	// 		SCALAR dtInitial = SCALAR(0.01)) {

	// 	StateVector<STATE_DIM, SCALAR> initialStateInternal = initialState;

	// 	boost::numeric::odeint::integrate_times(stepper_, systemFunction_, initialStateInternal,
	// 			&timeTrajectory.front(), &timeTrajectory.back()+1, dtInitial, Base::observer_.observeWrap);
	// }


	std::function<void (const Eigen::Matrix<SCALAR, STATE_DIM, 1>&, Eigen::Matrix<SCALAR, STATE_DIM, 1>&, SCALAR)> systemFunction_; //! the system function to integrate
	std::shared_ptr<internal::StepperBase<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>> integratorStepper_;
};

}
}

#endif /* CT_CORE_INTEGRATOR_H_ */
