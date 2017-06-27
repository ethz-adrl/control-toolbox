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


#ifndef CT_MPC_H_
#define CT_MPC_H_

#include <type_traits>

#include <ct/optcon/problem/OptConProblem.h>
#include <ct/optcon/solver/OptConSolver.h>

#include "MpcSettings.h"
#include "MpcTimeKeeper.h"

#include "policyhandler/PolicyHandler.h"
#include "policyhandler/default/PolicyHandlerILQG.h"

#include "timehorizon/MpcTimeHorizon.h"

#include <ct/optcon/ilqg/iLQGBase.hpp>

//#define DEBUG_PRINT_MPC	//! use this flag to enable debug printouts in the MPC implementation



namespace ct{
namespace optcon{


/** \defgroup MPC MPC
 *
 * \brief Model Predictive Control Module
 */

/**
 * \ingroup MPC
 *+
 * \brief Main MPC class.
 *
 * Created on: 28.02.2017
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch>
 *
 *	This MPC class allows to use any solver that derives from the OptConSolver base class in Model-Predictive-Control fashion.
 *	MPC will automatically construct the solver
 *
 *	Main assumptions:
 *	This MPC class is deliberately designed such that the time-keeping is managed by itself. The main assumption is that the controller
 *	which is designed here, gets applied to the system instantaneously after the run() call is executed. Furthermore, we assume that all Optimal Control Problems start at
 *	time zero. This also applies to the cost- and the constraint functionals which are to be provided by the user.
 *
 *	Sidenotes:
 *	between the calls to run(), the user can arbitrarily modify his cost-functions, etc. in order to change the problem.
 *
 *	@param OPTCON_SOLVER
 *		the optimal control solver to be employed, for example SLQ or DMS
 *
 */
template<typename OPTCON_SOLVER>
class MPC {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static const size_t STATE_DIM = OPTCON_SOLVER::STATE_D;
	static const size_t CONTROL_DIM = OPTCON_SOLVER::CONTROL_D;


	typedef typename OPTCON_SOLVER::Scalar_t Scalar_t;
	typedef typename OPTCON_SOLVER::Policy_t Policy_t;

	typedef OptConProblem<STATE_DIM, CONTROL_DIM, Scalar_t> OptConProblem_t;



	//! MPC solver constructor
	/*!
	 *
	 * @param problem
	 * 	the optimal control problem set up by the user
	 * @param solverSettings
	 * 	settings class/struct for the optimal control solver of choice. Be sure to tune the solver settings such that they are suitable for MPC!
	 * @param mpcsettings
	 *  mpc-specific settings, see class MpcSettings.h
	 * @param customPolicyHandler
	 * user-provided custom policy handler, which derives from base class 'PolicyHandler'.
	 *  If not specified, MPC will use one of its default implementations or throw an error if there is no default match.
	 * @param customTimeHorizon
	 *  user-provided custom time horizon strategy, which derives from base class 'MpcTimeHorizon'.
	 *  If not specified, MPC will use one of its default implementations or throw an error if there is no default match.
	 */
	MPC(
			const OptConProblem_t& problem,
			const typename OPTCON_SOLVER::Settings_t& solverSettings,
			const mpc_settings& mpcsettings = mpc_settings(),
			std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>> customPolicyHandler = nullptr,
			std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> customTimeHorizon = nullptr):

				solver_(problem, solverSettings),
				mpc_settings_(mpcsettings),
				dynamics_ (problem.getNonlinearSystem()->clone()),
				firstRun_(true),
				runCallCounter_(0),
				policyHandler_(new PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>())
	{

		// =========== INIT WARM START STRATEGY =============

		if(mpc_settings_.coldStart_ == false){

			if(customPolicyHandler)
			{
				std::cout << "Initializing MPC with a custom policy handler (warmstarter) provided by the user." << std::endl;
				policyHandler_ = customPolicyHandler;
			}
			else
			{
				if (std::is_base_of<iLQGBase<STATE_DIM, CONTROL_DIM, Scalar_t>, OPTCON_SOLVER>::value)
				{
					// default policy handler for standard discrete-time iLQG implementation
					policyHandler_ = std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>> (new PolicyHandlerILQG<STATE_DIM, CONTROL_DIM, Scalar_t>(solverSettings.dt));
				}
				else
				{
					throw std::runtime_error("ERROR in MPC Constructor -- no default warm start strategy available for the selected controller.");
				}
			}
		}


		// ==============  INIT MPC TIME HORIZON STRATEGY ==============
		if(customTimeHorizon)
		{
			std::cout << "Initializing MPC with a custom time-horizon strategy provided by the user." << std::endl;
			timeHorizonStrategy_ = customTimeHorizon;
		}
		else
		{
			const Scalar_t initTimeHorizon = solver_.getTimeHorizon();

			timeHorizonStrategy_ = std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> (new tpl::MpcTimeHorizon<Scalar_t>(mpc_settings_, initTimeHorizon));
		}


		// ==============  INIT MPC TIME KEEPER ==============
		timeKeeper_ = tpl::MpcTimeKeeper<Scalar_t>(timeHorizonStrategy_, mpc_settings_);
	}


	//! Allows access to the solver member, required mainly for unit testing.
	/*!
	 * @return reference to the optimal control problem solver
	 */
	OPTCON_SOLVER& getSolver() {return solver_;};


	//! Additional method to insert a custom time horizon strategy, independent from the constructor
	/*!
	 * @param timeHorizonStrategy
	 * 	the time horizon strategy provided by the user
	 */
	void setTimeHorizonStrategy(std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> timeHorizonStrategy){timeHorizonStrategy_ = timeHorizonStrategy;}


	//! set a new initial guess for the policy
	/**
	 * @param initGuess
	 */
	void setInitialGuess(const Policy_t& initGuess){
		solver_.setInitialGuess(initGuess);
		policyHandler_->setPolicy(initGuess);
		currentPolicy_ = initGuess;
	}


	//! Check if final time horizon for this task was reached
	bool timeHorizonReached() { return timeKeeper_.finalPointReached();}


	//! retrieve the time that elapsed since the first successful solve() call to an Optimal Control Problem
	/*!
	 * the returned time can be used externally, for example to update cost functions
	 * @return time elapsed
	 */
	const Scalar_t timeSinceFirstSuccessfulSolve()
	{
		return timeKeeper_.timeSinceFirstSuccessfulSolve();
	}


	//! perform forward integration of the measured system state, to compensate for expected or already occured time lags
	/*!
	 * State forward integration
	 * @param t_forward_start
	 * 	time where forward integration starts
	 * @param t_forward_stop
	 * 	time where forward integration stops
	 * @param x_start
	 * 	initial state for forward integration, gets overwritten with forward-integrated state
	 * @param forwardIntegrationController
	 *  (optional) external controller for forward integration
	 */
	void doPreIntegration(
			const Scalar_t& t_forward_start,
			const Scalar_t& t_forward_stop,
			core::StateVector<STATE_DIM, Scalar_t>& x_start,
			const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>> forwardIntegrationController = nullptr)
	{

		if(mpc_settings_.stateForwardIntegration_ == true)
		{

			if(forwardIntegrationController)
			{
				// ... either with a given third-party controller
				integrateForward(t_forward_start, t_forward_stop, x_start, forwardIntegrationController);
			}
			else
			{
				// ... or with the controller obtained from the solver (solution of last mpc-run).
				std::shared_ptr<Policy_t> prevController (new Policy_t(currentPolicy_));
				integrateForward(t_forward_start, t_forward_stop, x_start, prevController);
			}
		}
	}


	//! main MPC run method
	/*!
	 * @param x
	 * 	current system state
	 * @param x_ts
	 *  time stamp of the current state (external time in seconds)
	 * @param newPolicy
	 *  the new policy calculated by the MPC run() method based on above state, the timing info and the underlying OptConProblem
	 * @param newPolicy_ts
	 *  time stamp of the resulting policy. This indicates when the policy is supposed to start being applied, relative to
	 *  the user-provided state-timestamp x_ts.
	 * @param forwardIntegrationController
	 * 		optional input: in some scenarios, we wish to use a different kind controller for forward integrating the system than the one we are optimizing
	 * 		Such a controller can be handed over here as additional argument. If set to empty, MPC uses its own optimized controller from
	 * 		the last iteration.
	 * @return true if solve was successful, false otherwise.
	 */
	bool run(const core::StateVector<STATE_DIM, Scalar_t>& x,
			const Scalar_t x_ts,
			Policy_t& newPolicy,
			Scalar_t& newPolicy_ts,
			const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>> forwardIntegrationController = nullptr)
	{

#ifdef DEBUG_PRINT_MPC
		std::cout << "DEBUG_PRINT_MPC: started run() with state-timestamp " << x_ts << std::endl;
#endif //DEBUG_PRINT_MPC


		runCallCounter_++;

		// initialize the time-stamp for policy which is to be designed
		newPolicy_ts = x_ts;


		// local variables
		Scalar_t t_forward_start;
		Scalar_t t_forward_stop;

		const Scalar_t currTimeHorizon = solver_.getTimeHorizon();
		Scalar_t newTimeHorizon;

		core::StateVector<STATE_DIM, Scalar_t> x_start = x;


		if(firstRun_)
			timeKeeper_.initialize();


		timeKeeper_.startDelayMeasurement();


		timeKeeper_.computeNewTimings(currTimeHorizon, newTimeHorizon, t_forward_start, t_forward_stop);


		if(!firstRun_)
			doPreIntegration(t_forward_start, t_forward_stop, x_start, forwardIntegrationController);



		 // update the Optimal Control Solver with new time horizon and state information
		solver_.changeTimeHorizon(newTimeHorizon);

		solver_.changeInitialState(x_start);


		// Calculate new initial guess / warm-starting policy
		policyHandler_->designWarmStartingPolicy(t_forward_stop, newTimeHorizon, currentPolicy_, stateTrajectory_);

		// todo: remove this after through testing
		if(t_forward_stop < t_forward_start)
			throw std::runtime_error("ERROR: t_forward_stop < t_forward_start is impossible.");


		/**
		 * re-initialize the OptConSolver and solve the optimal control problem.
		 */
		solver_.setInitialGuess(currentPolicy_);

		bool solveSuccessful = solver_.solve();

		if(solveSuccessful){

			newPolicy_ts = newPolicy_ts + (t_forward_stop-t_forward_start);

			// get optimized policy and state trajectory from OptConSolver
			currentPolicy_ = solver_.getSolution();

			stateTrajectory_ = solver_.getStateTrajectory();

			// obtain the time which passed since the previous successful solve
			Scalar_t dtp = timeKeeper_.timeSincePreviousSuccessfulSolve();

			// post-truncation may be an option of the solve-call took longer than the estimated delay
			if(mpc_settings_.postTruncation_){

				if(dtp > t_forward_stop && !firstRun_)
				{
					// the time-difference to be account for by post-truncation
					Scalar_t dt_post_truncation = dtp-t_forward_stop;

#ifdef DEBUG_PRINT_MPC
					std::cout << "DEBUG_PRINT_MPC: additional post-truncation about "<< dt_post_truncation << " [sec]." << std::endl;
#endif //DEBUG_PRINT_MPC

					// the time which was effectively truncated away (e.g. discrete-time case)
					Scalar_t dt_truncated_eff;

					policyHandler_->truncateSolutionFront(dt_post_truncation, currentPolicy_, stateTrajectory_, dt_truncated_eff);

					// update policy timestamp with the truncated time
					newPolicy_ts += dt_truncated_eff;

				}
				else if (t_forward_stop >= dtp && !firstRun_)
				{
#ifdef DEBUG_PRINT_MPC
					std::cout << "DEBUG_PRINT_MPC: controller opt faster than pre-integration horizon. Consider tuning pre-integration. " << std::endl;
#endif //DEBUG_PRINT_MPC
				}

			} // post truncation

		} // solve successful


#ifdef DEBUG_PRINT_MPC
		std::cout << "DEBUG_PRINT_MPC: start timestamp outgoing policy: "<< newPolicy_ts << std::endl;
		std::cout << "DEBUG_PRINT_MPC: ended run() " << std::endl << std::endl;
#endif //DEBUG_PRINT_MPC

		// update policy result
		newPolicy = currentPolicy_;

		// stop the delay measurement. This needs to be the last method called in run().
		timeKeeper_.stopDelayMeasurement();

		// in the first run, the policy time-stamp needs to be shifted about the solving time
		if(firstRun_){
			newPolicy_ts = newPolicy_ts + timeKeeper_.getMeasuredDelay();
			firstRun_ = false;
		}

		return solveSuccessful;
	}


	//! reset the mpc problem and provide new problem time horizon (mandatory)
	void resetMpc(const Scalar_t& newTimeHorizon){

		firstRun_ = true;

		runCallCounter_ = 0;

		// reset the time horizon of the strategy
		timeHorizonStrategy_->updateInitialTimeHorizon(newTimeHorizon);

		solver_.changeTimeHorizon(newTimeHorizon);

		timeKeeper_.initialize();
	}


	//! update the mpc settings in all instances (main class, time keeper class, etc)
	/*!
	 * update the mpc settings in all instances
	 * @param settings
	 * 	the new mpc settings provided by the user.
	 */
	void updateSettings(const mpc_settings& settings) {
		mpc_settings_ = settings;
		timeKeeper_.updateSettings(settings);
		timeHorizonStrategy_->updateSettings(settings);
	}

	void setStateTrajectory(const core::StateTrajectory<STATE_DIM, Scalar_t>& x)
	{
		stateTrajectory_ = x;
	}

	//! obtain the solution state trajectory from the solver
	const core::StateTrajectory<STATE_DIM, Scalar_t> getStateTrajectory() const {return stateTrajectory_; }


	//! printout simple statistical data
	void printMpcSummary()
	{
		std::cout << std::endl;
		std::cout << "================ MPC Summary ================" << std::endl;
		std::cout << "Number of run() calls:\t\t\t" << runCallCounter_ << std::endl;

		if(mpc_settings_.measureDelay_)
		{
			std::cout << "Max measured solving time [sec]:\t" << timeKeeper_.getMaxMeasuredDelay() << std::endl;
			std::cout << "Min measured solving time [sec]:\t" << timeKeeper_.getMinMeasuredDelay() << std::endl;
			std::cout << "Total sum of meas. delay [sec]: \t" << timeKeeper_.getSummedDelay() << std::endl;
			std::cout << "Average measured delay [sec]:   \t" << timeKeeper_.getSummedDelay()/runCallCounter_ << std::endl;
		}
		else
		{
			std::cout << "Used fixed delay[sec]: \t" << 0.000001*mpc_settings_.fixedDelayUs_<< std::endl;
		}

		std::cout << "================ End Summary ================" << std::endl;
		std::cout << std::endl;
	}


private:


	//! state forward propagation (for delay compensation)
	/*!
	 * Perform forward integration about the given prediction horizon. Currently, this automatically uses adaptive integration.
	 * - uses an arbitrary controller given, which is important for hierarchical setups where the actual controller may be refined further
	 * - clones that controller, since it may be mistakenly used somewhere else otherwise.
	 * - output: the forward integrated state in x0
	 * @param startTime
	 * 	time where forward integration starts w.r.t. the current policy
	 * @param stopTime
	 *  time where forward integration stops w.r.t. the current policy
	 * @param state
	 *  state to be forward propagated
	 * @param controller
	 *  the controller to be used for forward propagation
	 */
	void integrateForward(
			const Scalar_t startTime,
			const Scalar_t stopTime,
			core::StateVector<STATE_DIM, Scalar_t>& state,
			const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>>& controller)
	{
		dynamics_->setController(controller);

		Scalar_t dtInit = 0.0001;

		// create temporary integrator object
		core::IntegratorRK4<STATE_DIM, Scalar_t> newInt (dynamics_);

		// adaptive pre-integration
		newInt.integrate_adaptive(state, startTime, stopTime, dtInit);
	}


	OPTCON_SOLVER solver_;	//! optimal control solver employed for mpc

	Policy_t currentPolicy_;	//! currently optimal policy, initial guess respectively

	std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>> policyHandler_;	//! policy handler, which takes care of warm-starting

	std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> timeHorizonStrategy_;	//! time horizon strategy, e.g. receding horizon optimal control

	tpl::MpcTimeKeeper<Scalar_t> timeKeeper_;	//! time keeper

	mpc_settings mpc_settings_;	//! mpc settings

	bool firstRun_;	//! true for first run

	typename OPTCON_SOLVER::OptConProblem_t::DynamicsPtr_t dynamics_;	//! dynamics instance for forward integration

	core::StateTrajectory<STATE_DIM, Scalar_t> stateTrajectory_;	//! state solution trajectory

	size_t runCallCounter_;	//! counter which gets incremented at every call of the run() method

};


}	//namespace optcon
}	//namespace ct

#endif /* CT_MPC_H_ */
