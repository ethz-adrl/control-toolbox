/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename OPTCON_SOLVER>
MPC<OPTCON_SOLVER>::MPC(const OptConProblem_t& problem,
    const typename OPTCON_SOLVER::Settings_t& solverSettings,
    const mpc_settings& mpcsettings,
    std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>> customPolicyHandler,
    std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> customTimeHorizon)
    :

      solver_(problem, solverSettings),
      mpc_settings_(mpcsettings),
      dynamics_(problem.getNonlinearSystem()->clone()),
      forwardIntegrator_(dynamics_, mpcsettings.stateForwardIntegratorType_),
      firstRun_(true),
      runCallCounter_(0),
      policyHandler_(new PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>())
{
    checkSettings(mpcsettings);

    // =========== INIT WARM START STRATEGY =============

    if (mpc_settings_.coldStart_ == false)
    {
        if (customPolicyHandler)
        {
            std::cout << "Initializing MPC with a custom policy handler (warmstarter) provided by the user."
                      << std::endl;
            policyHandler_ = customPolicyHandler;
        }
        else
        {
            if (std::is_base_of<NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, Scalar_t>, OPTCON_SOLVER>::value)
            {
                // default policy handler for standard discrete-time iLQG implementation
                policyHandler_ = std::shared_ptr<PolicyHandler<Policy_t, STATE_DIM, CONTROL_DIM, Scalar_t>>(
                    new StateFeedbackPolicyHandler<STATE_DIM, CONTROL_DIM, Scalar_t>(solverSettings.dt));
            }
            else
            {
                throw std::runtime_error(
                    "ERROR in MPC Constructor -- no default warm start strategy available for the selected "
                    "controller.");
            }
        }
    }


    // ==============  INIT MPC TIME HORIZON STRATEGY ==============
    if (customTimeHorizon)
    {
        std::cout << "Initializing MPC with a custom time-horizon strategy provided by the user." << std::endl;
        timeHorizonStrategy_ = customTimeHorizon;
    }
    else
    {
        const Scalar_t initTimeHorizon = solver_.getTimeHorizon();

        timeHorizonStrategy_ = std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>>(
            new tpl::MpcTimeHorizon<Scalar_t>(mpc_settings_, initTimeHorizon));
    }


    // ==============  INIT MPC TIME KEEPER ==============
    timeKeeper_ = tpl::MpcTimeKeeper<Scalar_t>(timeHorizonStrategy_, mpc_settings_);
}


template <typename OPTCON_SOLVER>
OPTCON_SOLVER& MPC<OPTCON_SOLVER>::getSolver()
{
    return solver_;
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::setTimeHorizonStrategy(std::shared_ptr<tpl::MpcTimeHorizon<Scalar_t>> timeHorizonStrategy)
{
    timeHorizonStrategy_ = timeHorizonStrategy;
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::setInitialGuess(const Policy_t& initGuess)
{
    solver_.setInitialGuess(initGuess);
    policyHandler_->setPolicy(initGuess);
    currentPolicy_ = initGuess;
}


template <typename OPTCON_SOLVER>
bool MPC<OPTCON_SOLVER>::timeHorizonReached()
{
    return timeKeeper_.finalPointReached();
}


template <typename OPTCON_SOLVER>
const typename MPC<OPTCON_SOLVER>::Scalar_t MPC<OPTCON_SOLVER>::timeSinceFirstSuccessfulSolve(const Scalar_t& extTime)
{
    return timeKeeper_.timeSinceFirstSuccessfulSolve(extTime);
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::doForwardIntegration(const Scalar_t& t_forward_start,
    const Scalar_t& t_forward_stop,
    core::StateVector<STATE_DIM, Scalar_t>& x_start,
    const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>> forwardIntegrationController)
{
    if (mpc_settings_.stateForwardIntegration_ == true)
    {
        if (forwardIntegrationController)
        {
            // ... either with a given third-party controller
            integrateForward(t_forward_start, t_forward_stop, x_start, forwardIntegrationController);
        }
        else
        {
            // ... or with the controller obtained from the solver (solution of last mpc-run).
            std::shared_ptr<Policy_t> prevController(new Policy_t(currentPolicy_));
            integrateForward(t_forward_start, t_forward_stop, x_start, prevController);
        }
    }
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::prepareIteration(const Scalar_t& extTime)
{
#ifdef DEBUG_PRINT_MPC
    std::cout << "DEBUG_PRINT_MPC: started to prepare MPC iteration() " << std::endl;
#endif  //DEBUG_PRINT_MPC

    runCallCounter_++;

    const Scalar_t currTimeHorizon = solver_.getTimeHorizon();
    Scalar_t newTimeHorizon;


    if (firstRun_)
        timeKeeper_.initialize();

    timeKeeper_.computeNewTimings(extTime, currTimeHorizon, newTimeHorizon, t_forward_start_, t_forward_stop_);

    // update the Optimal Control Solver with new time horizon and state information
    solver_.changeTimeHorizon(newTimeHorizon);


    // Calculate new initial guess / warm-starting policy
    policyHandler_->designWarmStartingPolicy(t_forward_stop_, newTimeHorizon, currentPolicy_);

    // todo: remove this after through testing
    if (t_forward_stop_ < t_forward_start_)
        throw std::runtime_error("ERROR: t_forward_stop < t_forward_start is impossible.");


    /**
	 * re-initialize the OptConSolver and solve the optimal control problem.
	 */
    solver_.setInitialGuess(currentPolicy_);

    solver_.prepareMPCIteration();
}


template <typename OPTCON_SOLVER>
bool MPC<OPTCON_SOLVER>::finishIteration(const core::StateVector<STATE_DIM, Scalar_t>& x,
    const Scalar_t x_ts,
    Policy_t& newPolicy,
    Scalar_t& newPolicy_ts,
    const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>> forwardIntegrationController)
{
#ifdef DEBUG_PRINT_MPC
    std::cout << "DEBUG_PRINT_MPC: started mpc finish Iteration() with state-timestamp " << x_ts << std::endl;
#endif  //DEBUG_PRINT_MPC

    timeKeeper_.startDelayMeasurement(x_ts);

    // initialize the time-stamp for policy which is to be designed
    newPolicy_ts = x_ts;

    core::StateVector<STATE_DIM, Scalar_t> x_start = x;

    if (!firstRun_)
        doForwardIntegration(t_forward_start_, t_forward_stop_, x_start, forwardIntegrationController);


    solver_.changeInitialState(x_start);

    bool solveSuccessful = solver_.finishMPCIteration();

    if (solveSuccessful)
    {
        newPolicy_ts = newPolicy_ts + (t_forward_stop_ - t_forward_start_);

        // get optimized policy and state trajectory from OptConSolver
        currentPolicy_ = solver_.getSolution();

        // obtain the time which passed since the previous successful solve
        Scalar_t dtp = timeKeeper_.timeSincePreviousSuccessfulSolve(x_ts);

        // post-truncation may be an option of the solve-call took longer than the estimated delay
        if (mpc_settings_.postTruncation_)
        {
            if (dtp > t_forward_stop_ && !firstRun_)
            {
                // the time-difference to be account for by post-truncation
                Scalar_t dt_post_truncation = dtp - t_forward_stop_;

#ifdef DEBUG_PRINT_MPC
                std::cout << "DEBUG_PRINT_MPC: additional post-truncation about " << dt_post_truncation << " [sec]."
                          << std::endl;
#endif  //DEBUG_PRINT_MPC

                // the time which was effectively truncated away (e.g. discrete-time case)
                Scalar_t dt_truncated_eff;

                policyHandler_->truncateSolutionFront(dt_post_truncation, currentPolicy_, dt_truncated_eff);

                // update policy timestamp with the truncated time
                newPolicy_ts += dt_truncated_eff;
            }
            else if (t_forward_stop_ >= dtp && !firstRun_)
            {
#ifdef DEBUG_PRINT_MPC
                std::cout << "DEBUG_PRINT_MPC: controller opt faster than pre-integration horizon. Consider tuning "
                             "pre-integration. "
                          << std::endl;
#endif  //DEBUG_PRINT_MPC
            }

        }  // post truncation

    }  // solve successful


#ifdef DEBUG_PRINT_MPC
    std::cout << "DEBUG_PRINT_MPC: start timestamp outgoing policy: " << newPolicy_ts << std::endl;
    std::cout << "DEBUG_PRINT_MPC: ended finishIteration() " << std::endl << std::endl;
#endif  //DEBUG_PRINT_MPC

    // update policy result
    newPolicy = currentPolicy_;

    // stop the delay measurement. This needs to be the last method called in finishIteration().
    timeKeeper_.stopDelayMeasurement(x_ts);

    // in the first run, the policy time-stamp needs to be shifted about the solving time
    if (firstRun_)
    {
        newPolicy_ts = newPolicy_ts + timeKeeper_.getMeasuredDelay();
        firstRun_ = false;
    }

    return solveSuccessful;
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::resetMpc(const Scalar_t& newTimeHorizon)
{
    firstRun_ = true;

    runCallCounter_ = 0;

    // reset the time horizon of the strategy
    timeHorizonStrategy_->updateInitialTimeHorizon(newTimeHorizon);

    solver_.changeTimeHorizon(newTimeHorizon);

    timeKeeper_.initialize();
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::updateSettings(const mpc_settings& settings)
{
    checkSettings(settings);

    if (settings.stateForwardIntegratorType_ != mpc_settings_.stateForwardIntegratorType_)
        forwardIntegrator_ = ct::core::Integrator<STATE_DIM, Scalar_t>(dynamics_, settings.stateForwardIntegratorType_);

    mpc_settings_ = settings;
    timeKeeper_.updateSettings(settings);
    timeHorizonStrategy_->updateSettings(settings);
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::printMpcSummary()
{
    std::cout << std::endl;
    std::cout << "================ MPC Summary ================" << std::endl;
    std::cout << "Number of MPC calls:\t\t\t" << runCallCounter_ << std::endl;

    if (mpc_settings_.measureDelay_)
    {
        std::cout << "Max measured solving time [sec]:\t" << timeKeeper_.getMaxMeasuredDelay() << std::endl;
        std::cout << "Min measured solving time [sec]:\t" << timeKeeper_.getMinMeasuredDelay() << std::endl;
        std::cout << "Total sum of meas. delay [sec]: \t" << timeKeeper_.getSummedDelay() << std::endl;
        std::cout << "Average measured delay [sec]:   \t" << timeKeeper_.getSummedDelay() / runCallCounter_
                  << std::endl;
    }
    else
    {
        std::cout << "Used fixed delay[sec]: \t" << 0.000001 * mpc_settings_.fixedDelayUs_ << std::endl;
    }

    std::cout << "================ End Summary ================" << std::endl;
    std::cout << std::endl;
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::integrateForward(const Scalar_t startTime,
    const Scalar_t stopTime,
    core::StateVector<STATE_DIM, Scalar_t>& state,
    const std::shared_ptr<core::Controller<STATE_DIM, CONTROL_DIM, Scalar_t>>& controller)
{
    dynamics_->setController(controller);

    int nSteps = std::max(0, (int)std::lround((stopTime - startTime) / mpc_settings_.stateForwardIntegration_dt_));

    // adaptive pre-integration
    forwardIntegrator_.integrate_n_steps(state, startTime, nSteps, mpc_settings_.stateForwardIntegration_dt_);
}


template <typename OPTCON_SOLVER>
void MPC<OPTCON_SOLVER>::checkSettings(const mpc_settings& settings)
{
    // if external timing is active, internal delay measurement must be turned off
    if (settings.useExternalTiming_ && settings.measureDelay_)
        throw std::runtime_error(
            "MPC: measuring delay internally contradicts using external Timing. Switch off one of those options.");
}

}  //namespace optcon
}  //namespace ct
