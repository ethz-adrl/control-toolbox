/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackPolicyHandler<STATE_DIM, CONTROL_DIM, SCALAR>::StateFeedbackPolicyHandler(const SCALAR& dt) : dt_(dt)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackPolicyHandler<STATE_DIM, CONTROL_DIM, SCALAR>::~StateFeedbackPolicyHandler()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackPolicyHandler<STATE_DIM, CONTROL_DIM, SCALAR>::designWarmStartingPolicy(const SCALAR& delay,
    const SCALAR& newTimeHorizon,
    StateFeedbackController_t& policy)
{
    // get the current reference trajectories from the StateFeedbackController
    core::FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& FeedbackTraj = policy.getFeedbackTrajectory();
    core::ControlTrajectory<CONTROL_DIM, SCALAR>& FeedForwardTraj = policy.getFeedforwardTrajectory();
    core::StateTrajectory<STATE_DIM, SCALAR>& StateRefTraj = policy.getReferenceStateTrajectory();

    // current number of discrete elements
    int currentSize = FeedForwardTraj.size();

    // compute new controller length as a function of the time horizon
    int Kn_new = std::max(1, (int)std::lround(newTimeHorizon / dt_));

    // compute number indices to be shifted. Note: it does not make sense to shift more entries than are available
    int num_di = FeedForwardTraj.getIndexFromTime(delay);
    num_di = std::min(num_di, currentSize - 1);


#ifdef DEBUG_POLICYHANDLER
    std::cout << "DEBUG_POLICYHANDLER: Controller shifting: " << std::endl
              << "delay: " << delay << "  newT: " << newTimeHorizon << std::endl
              << " new Discrete Controller has:  " << std::endl
              << Kn_new << " control elements, shifted about " << num_di << " elements." << std::endl
              << Kn_new + 1 << " state elements, shifted about " << num_di << " elements." << std::endl;
#endif


    // Step 1 - Truncate Front: remove first 'num_di' elements from controller and shift time accordingly
    if (num_di > 0)
    {
        FeedForwardTraj.eraseFront(num_di, num_di * dt_);
        FeedbackTraj.eraseFront(num_di, num_di * dt_);
        StateRefTraj.eraseFront(num_di, num_di * dt_);
        currentSize -= num_di;
    }


    // Step 2 - Resize overall controller
    if (Kn_new > currentSize)
    {
        //extend at back with constant value taken from last element
        bool timeIsRelative = true;
        for (int i = 0; i < Kn_new - currentSize; i++)
        {
            FeedbackTraj.push_back(FeedbackTraj.back(), dt_, timeIsRelative);
            FeedForwardTraj.push_back(FeedForwardTraj.back(), dt_, timeIsRelative);
            StateRefTraj.push_back(StateRefTraj.back(), dt_, timeIsRelative);
        }
    }
    else if (Kn_new < currentSize)
    {
        // remove elements from back
        for (int i = 0; i < currentSize - Kn_new; i++)
        {
            FeedbackTraj.pop_back();
            FeedForwardTraj.pop_back();
            StateRefTraj.pop_back();
        }
    }

    // safety check, which should never be entered
    if (FeedForwardTraj.size() == 0)
    {
        throw std::runtime_error("ERROR in StateFeedbackPolicyHandler.h: new policy should not have size 0.");
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackPolicyHandler<STATE_DIM, CONTROL_DIM, SCALAR>::truncateSolutionFront(const SCALAR& delay,
    StateFeedbackController_t& policy,
    SCALAR& effectivelyTruncated)
{
    // current controller length
    size_t currentSize = policy.getFeedforwardTrajectory().size();

    size_t num_di = policy.getFeedforwardTrajectory().getIndexFromTime(delay);
    num_di = std::min(num_di, currentSize - 1);

    effectivelyTruncated = num_di * dt_;

#ifdef DEBUG_POLICYHANDLER
    std::cout << "DEBUG_WARMSTART: Current Controller Size:  " << currentSize << " elements." << std::endl;
    std::cout << "DEBUG_WARMSTART: Controller truncation: truncation about " << num_di << " elements." << std::endl;
    std::cout << "DEBUG_WARMSTART: Controller new size: " << currentSize - num_di << " elements." << std::endl;
#endif

    // remove first num_di elements from controller
    if (num_di > 0 && num_di < currentSize)
    {
        policy.getFeedbackTrajectory().eraseFront(num_di, effectivelyTruncated);
        policy.getFeedforwardTrajectory().eraseFront(num_di, effectivelyTruncated);
        policy.getReferenceStateTrajectory().eraseFront(num_di, effectivelyTruncated);
    }
}

}  // namespace optcon
}  // namespace ct
