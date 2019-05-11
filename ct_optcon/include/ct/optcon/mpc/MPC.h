/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include <type_traits>

#include <ct/optcon/problem/ContinuousOptConProblem.h>
#include <ct/optcon/solver/OptConSolver.h>

#include "MpcSettings.h"
#include "MpcTimeKeeper.h"

#include "policyhandler/PolicyHandler.h"
#include "timehorizon/MpcTimeHorizon.h"

#include <ct/optcon/solver/NLOptConSolver.hpp>
#include "policyhandler/default/StateFeedbackPolicyHandler.h"

//#define DEBUG_PRINT_MPC	//! use this flag to enable debug printouts in the MPC implementation


namespace ct {
namespace optcon {


/** \defgroup MPC MPC
 *
 * \brief Model Predictive Control Module
 */

/**
 * \ingroup MPC
 *+
 * \brief Main MPC class.
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
template <typename OPTCON_SOLVER>
class MPC
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = OPTCON_SOLVER::STATE_D;
    static const size_t CONTROL_DIM = OPTCON_SOLVER::CONTROL_D;

    static const size_t P_DIM = OPTCON_SOLVER::POS_DIM;
    static const size_t V_DIM = OPTCON_SOLVER::VEL_DIM;

    using Scalar_t = typename OPTCON_SOLVER::Scalar_t;
    using Policy_t = typename OPTCON_SOLVER::Policy_t;
    using OptConProblem_t = ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, Scalar_t>;


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
    MPC(const OptConProblem_t& problem,
        const typename OPTCON_SOLVER::Settings_t& solverSettings,
        const mpc_settings& mpcsettings = mpc_settings(),
        std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>> customPolicyHandler = nullptr,
        std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> customTimeHorizon = nullptr);


    //! Allows access to the solver member, required mainly for unit testing.
    /*!
	 * @return reference to the optimal control problem solver
	 */
    OPTCON_SOLVER& getSolver();


    //! Additional method to insert a custom time horizon strategy, independent from the constructor
    /*!
	 * @param timeHorizonStrategy
	 * 	the time horizon strategy provided by the user
	 */
    void setTimeHorizonStrategy(std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> timeHorizonStrategy);

    //! set a new initial guess for the policy
    /**
	 * @param initGuess
	 */
    void setInitialGuess(const Policy_t& initGuess);

    //! Check if final time horizon for this task was reached
    bool timeHorizonReached();


    //! retrieve the time that elapsed since the first successful solve() call to an Optimal Control Problem
    /*!
	 * @param the external time stamp
	 * @return time elapsed, the returned time can be used externally, for example to update cost functions
	 */
    const Scalar_t timeSinceFirstSuccessfulSolve(const Scalar_t& extTime);


    //! perform forward integration of the measured system state, to compensate for expected or already occurred time lags
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
	 *
	 *  \warning The effect of the integration will vanish one the MPC frequency is higher than the sampling frequency
	 */
    void doForwardIntegration(const Scalar_t& t_forward_start,
        const Scalar_t& t_forward_stop,
        core::StateVector<STATE_DIM, Scalar_t>& x_start,
        const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>> forwardIntegrationController =
            nullptr);


    /*!
     * Prepare MPC iteration
     * @param ext_ts the current external time
     */
    void prepareIteration(const Scalar_t& ext_ts);


    //! finish MPC iteration
    /*!
   	 * @param x
   	 * 	current system state
   	 * @param x_ts
   	 *  time stamp of the current state (external time in seconds)
   	 * @param newPolicy
   	 *  the new policy calculated based on above state, the timing info and the underlying OptConProblem
   	 * @param newPolicy_ts
   	 *  time stamp of the resulting policy. This indicates when the policy is supposed to start being applied, relative to
   	 *  the user-provided state-timestamp x_ts.
   	 * @param forwardIntegrationController
   	 * 		optional input: in some scenarios, we wish to use a different kind controller for forward integrating the system than the one we are optimizing
   	 * 		Such a controller can be handed over here as additional argument. If set to empty, MPC uses its own optimized controller from
   	 * 		the last iteration, thus assuming perfect control trajectory tracking.
   	 * @return true if solve was successful, false otherwise.
   	 */
    bool finishIteration(const core::StateVector<STATE_DIM, Scalar_t>& x,
        const Scalar_t x_ts,
        Policy_t& newPolicy,
        Scalar_t& newPolicy_ts,
        const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>> forwardIntegrationController =
            nullptr);


    //! reset the mpc problem and provide new problem time horizon (mandatory)
    void resetMpc(const Scalar_t& newTimeHorizon);


    //! update the mpc settings in all instances (main class, time keeper class, etc)
    /*!
	 * update the mpc settings in all instances
	 * @param settings
	 * 	the new mpc settings provided by the user.
	 */
    void updateSettings(const mpc_settings& settings);


    //! printout simple statistical data
    void printMpcSummary();


private:
    //! state forward propagation (for delay compensation)
    /*!
	 * Perform forward integration about the given prediction horizon.
	 * - uses an arbitrary controller given, which is important for hierarchical setups where the actual controller may be refined further
	 * @param startTime
	 * 	time where forward integration starts w.r.t. the current policy
	 * @param stopTime
	 *  time where forward integration stops w.r.t. the current policy
	 * @param state
	 *  state to be forward propagated
	 * @param controller
	 *  the controller to be used for forward propagation
	 */
    void integrateForward(const Scalar_t startTime,
        const Scalar_t stopTime,
        core::StateVector<STATE_DIM, Scalar_t>& state,
        const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>>& controller);

    void checkSettings(const mpc_settings& settings);

    //! timings for pre-integration
    Scalar_t t_forward_start_;
    Scalar_t t_forward_stop_;

    //! optimal control solver employed for mpc
    OPTCON_SOLVER solver_;

    //! mpc settings
    mpc_settings mpc_settings_;

    //! dynamics instance for forward integration
    typename OPTCON_SOLVER::OptConProblem_t::DynamicsPtr_t dynamics_;

    //! integrator for forward integration
    ct::core::Integrator<STATE_DIM, Scalar_t> forwardIntegrator_;

    //! true for first run
    bool firstRun_;

    //! counter which gets incremented at every call of the run() method
    size_t runCallCounter_;

    //! policy handler, which takes care of warm-starting
    std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>> policyHandler_;

    //! currently optimal policy, initial guess respectively
    Policy_t currentPolicy_;

    //! time horizon strategy, e.g. receding horizon optimal control
    std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> timeHorizonStrategy_;

    //! time keeper
    tpl::MpcTimeKeeper<Scalar_t> timeKeeper_;
};


}  //namespace optcon
}  //namespace ct
