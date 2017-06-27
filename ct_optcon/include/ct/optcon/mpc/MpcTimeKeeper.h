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

#ifndef CT_MPCTIMEKEEPER_H_
#define CT_MPCTIMEKEEPER_H_

#include "MpcSettings.h"
#include "timehorizon/MpcTimeHorizon.h"

#include <ct/optcon/problem/OptConProblem.h>

/**
 * define the following flag for debugging the MPC time keeper
 */
//#define DEBUG_PRINT_TIMEKEEPER

namespace ct{
namespace optcon{
namespace tpl{


//! Time Keeper Class for Model Predictive Control
/*!
 * - uses an internal clock for bookkeeping of times between MPC clycles
 * - this class is based on the assumption that all Optimal Control Problems start at t = 0. All times computed here are
 * expressed as relative times w.r.t. the start of the Optimal Control Problem.
 * - calculates the required shifting times between iterations and due to forward integration (initial state prediction)
 * - update Time Horizons accordingly
 */
template <typename SCALAR = double>
class MpcTimeKeeper{

public:

	//! Standard Constructor MpcTimeKeeper
	/*!
	 * this constructor should not be used by the normal user. Todo: get rid of MpcTimeKeeper default constructor
	 */
	MpcTimeKeeper(){}


	//! Constructor for Mpc Time Keeper class
	/*!
	 *
	 * @param timeHorizonStrategy the user-specified time horizon strategy, for example fixed time horizon
	 * @param mpc_settings the mpc_settings as specified by the user
	 */
	MpcTimeKeeper(std::shared_ptr<MpcTimeHorizon<SCALAR>> timeHorizonStrategy, const mpc_settings& mpc_settings):
		mpc_settings_(mpc_settings),
		initialized_(false),
		finalPointReached_(false),
		lastMeasuredDelay_(0.0),
		maxDelayMeasured_(0.0),
		minDelayMeasured_(std::numeric_limits<SCALAR>::max()),
		summedDelay_(0.0),
		timeHorizonStrategy_(timeHorizonStrategy),
		firstRun_(true)
	{}


	//! initialize the Mpc Time Keeper (mandatory)
	/*!
	 * resets the TimeKeeper, needs to be called whenever MPC starts over from the beginning.
	 */
	void initialize() {
		firstRun_ = true;

		timer_.reset();
		lastSolveTimer_.reset();
		firstSolveTimer_.reset();

		lastMeasuredDelay_ = 0.0;
		maxDelayMeasured_ = 0.0;
		minDelayMeasured_ = std::numeric_limits<SCALAR>::max();
		summedDelay_ = 0.0;
		finalPointReached_ = false;

		initialized_ = true;
	}


	//! compute new mpc timings, based on current time horizon and the given time horizon strategy
	/*!
	 *
	 * @param current_T
	 * 	the currently active problem time horizon
	 * @param new_T
	 * 	the new, updated problem time horizon, which gets computed by the time horizon strategy
	 * @param t_forw_start
	 * 	time where to start the forward propagation of the state measurement based on delays and currently optimal policy
	 * @param t_forw_stop
	 * 	time where to stop the forward propagation of the state measurement based on delays and currently optimal policy
	 */
	void computeNewTimings(
			const SCALAR current_T,
			SCALAR& new_T,
			SCALAR& t_forw_start,	// relative time to start forward integration
			SCALAR& t_forw_stop)	// relative time to stop forward integration
	{
		if(initialized_ == false)
			throw std::runtime_error("Error in MPC time keeper: cannot update timings if MpcTimeKeeper not properly initialized.");


		SCALAR timeSinceEndedLastSolve;
		SCALAR timeSinceEndedFirstSolve;

		if(!firstRun_){
			lastSolveTimer_.stop();
			firstSolveTimer_.stop();
			timeSinceEndedLastSolve  = lastSolveTimer_.getElapsedTime();
			timeSinceEndedFirstSolve = firstSolveTimer_.getElapsedTime();
		}
		else{
			timeSinceEndedLastSolve  = 0.0;
			timeSinceEndedFirstSolve = 0.0;
		}


		// starting time of forward prediction, relative to previous controller
		t_forw_start = timeSinceEndedLastSolve ;


		/**
		 * estimate the delay from planning, etc.
		 */
		SCALAR delayToApply = computeDelayToApply();


		// stopping time relative to previous controller/run
		if(!firstRun_)
			t_forw_stop = t_forw_start + delayToApply;
		else
			t_forw_stop = t_forw_start;


		/**
		 * check for compliance of t_forward_stop and t_forward_start with time horizon of the current controller
		 */
		if (t_forw_start > current_T){
			std::cerr << "WARNING: forward integration start time cannot be bigger than last time horizon. Truncating forward integration time." << std::endl;
			t_forw_start = current_T;
		}
		if(t_forw_stop > current_T){
			std::cerr << "WARNING: forward integration stop time cannot be bigger than last time horizon. Truncating forward integration time." << std::endl;
			t_forw_stop = current_T;
		}


		finalPointReached_ = timeHorizonStrategy_->computeNewTimeHorizon(timeSinceEndedFirstSolve, t_forw_stop, new_T);


#ifdef DEBUG_PRINT_TIMEKEEPER
		std::cout << "DEBUG_PRINT_TIMEKEEPER: Time since first solve(): "<< timeSinceEndedFirstSolve << std::endl;
		std::cout << "DEBUG_PRINT_TIMEKEEPER: Time since last solve(): "<< timeSinceEndedLastSolve << std::endl;
		std::cout << "DEBUG_PRINT_TIMEKEEPER: t_forward_start: " << t_forw_start << std::endl;
		std::cout << "DEBUG_PRINT_TIMEKEEPER: t_forward_stop: " << t_forw_stop << std::endl;
		std::cout << "DEBUG_PRINT_TIMEKEEPER: New Time Horizon: " << new_T << std::endl;
		std::cout << "DEBUG_PRINT_TIMEKEEPER: final point reached_: " << finalPointReached_ << std::endl;
#endif

	}


	//!  query this in order to find out if the final time horizon has been reached.
	/*!
	 * May be used to trigger corresponding events, such as stopping the control loop.
	 * @return bool, true if final time horizon has been reached
	 */
	const bool finalPointReached() const {return finalPointReached_;}


	//! update mpc settings
	/*!
	 * @param settings
	 * 	the new settings to be handed over
	 */
	void updateSettings(const mpc_settings& settings) {mpc_settings_ = settings;}


	//! start measuring time elapsed during planning / solving the optimal control problem
	void startDelayMeasurement() {
		if (mpc_settings_.measureDelay_)
		{
			timer_.start();
		}
	}


	//! stop measuring time elapsed during solving the optimal control problem
	void stopDelayMeasurement(){
		if (mpc_settings_.measureDelay_)
		{
			timer_.stop();
			lastMeasuredDelay_ = timer_.getElapsedTime();
			maxDelayMeasured_ = std::max(maxDelayMeasured_, lastMeasuredDelay_);
			minDelayMeasured_ = std::min(minDelayMeasured_, lastMeasuredDelay_);
			summedDelay_ += lastMeasuredDelay_;

			if(lastMeasuredDelay_ < 0)
				throw std::runtime_error("Fatal: measured delay cannot be < 0");

#ifdef DEBUG_PRINT_TIMEKEEPER
			std::cout << "Measured delay during Solution: " << lastMeasuredDelay_ << " seconds" << std::endl;
			std::cout << "Max. measured delay during Solution: " << maxDelayMeasured_ << " seconds" << std::endl;
			std::cout << "Min. measured delay during Solution: " << minDelayMeasured_ << " seconds" << std::endl;
#endif
		}

		lastSolveTimer_.start();	// to measure how much time passed since last successful solve()

		if(firstRun_){
			firstSolveTimer_.start();	// timer for measuring how much time elapsed since the first successful plan
			firstRun_ = false;
		}
	}


	//! retrieve the time that elapsed since the last successful solve() call to an Optimal Control Problem
	/*!
	 * the returned time can be used to synchronize the calls to optimal control problems
	 * @return time elapsed
	 */
	SCALAR timeSincePreviousSuccessfulSolve() {
		if(firstRun_)
			return 0.0;
		else{
			lastSolveTimer_.stop();
			return lastSolveTimer_.getElapsedTime();
		}
	}


	//! retrieve the time that elapsed since the first successful solve() call to an Optimal Control Problem
	/*!
	 * the returned time can be used externally, for example to update cost functions
	 * @return time elapsed
	 */
	const SCALAR timeSinceFirstSuccessfulSolve() {
		if(firstRun_)
			return 0.0;
		else{
			firstSolveTimer_.stop();
			return firstSolveTimer_.getElapsedTime();
		}
	}

	//! obtain the delay which was measured during solving the optimal control problem
	const SCALAR& getMeasuredDelay() const {return lastMeasuredDelay_;}

	//! get the maximum measured delay (maximum over all cycles)
	const SCALAR& getMaxMeasuredDelay() const {return maxDelayMeasured_;}

	//! get the smallest measured delay (minimum over all cycles)
	const SCALAR& getMinMeasuredDelay() const {return minDelayMeasured_;}

	//! get the sum of all measured delays
	const SCALAR& getSummedDelay() const {return summedDelay_;}

private:

	//! computes the delay to be applied to the policy based on fixed delays and measured delays
	/*!
	 * The delay to be applied consinst of a fixed-time delay specified by the user (for example communication delays)
	 * and a variable delay, which is based on a delay-measurement, which captures the optimal control problem solving times.
	 * The delay to be applied is the sum of fixed and variable components.
	 * @return delay to be applied
	 */
	SCALAR computeDelayToApply() {

		SCALAR fixedDelay = 1e-6 * mpc_settings_.additionalDelayUs_;
		SCALAR variableDelay = 0.0;

		if (mpc_settings_.measureDelay_)	// use variable, measured delay
			variableDelay = mpc_settings_.delayMeasurementMultiplier_ * lastMeasuredDelay_;
		else // using fixed delay
			fixedDelay += 1e-6 * mpc_settings_.fixedDelayUs_;

		return  fixedDelay + variableDelay;
	}


	mpc_settings mpc_settings_;

	bool initialized_;

	bool finalPointReached_;			//! flag indicating that the time horizon has been hit

	core::tpl::Timer<SCALAR> timer_;					//! timer for measuring the time elapsed during planning, internal, relative time
	core::tpl::Timer<SCALAR> lastSolveTimer_;		//! timer to measure how much time elapsed since the last finished solve
	core::tpl::Timer<SCALAR> firstSolveTimer_; 		//! timer for measuring how much time elapsed since the first successful plan

	SCALAR lastMeasuredDelay_;		//! last delay in planning, internal, relative time
	SCALAR maxDelayMeasured_;		//! the max. delay occurred due to planning
	SCALAR minDelayMeasured_;		//! the min. delay occurred due to planning
	SCALAR summedDelay_;			//! sum of all delays measured

	std::shared_ptr<MpcTimeHorizon<SCALAR>> timeHorizonStrategy_;	//! time horizon strategy specified by the user, e.g. constant receding horizon

	bool firstRun_;						//! set to true if first run active
};


} // namespace tpl

typedef tpl::MpcTimeKeeper<double> MpcTimeKeeper;

} // namespace optcon
} // namespace ct



#endif /* MPCTIMEKEEPER_H_ */
