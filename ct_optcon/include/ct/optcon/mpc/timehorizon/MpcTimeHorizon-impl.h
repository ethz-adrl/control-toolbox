/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {
namespace tpl {

template <typename SCALAR>
MpcTimeHorizon<SCALAR>::MpcTimeHorizon(const mpc_settings& settings, const SCALAR& initialTimeHorizon)
    : mpc_settings_(settings), initialTimeHorizon_(initialTimeHorizon)
{
}

template <typename SCALAR>
MpcTimeHorizon<SCALAR>::~MpcTimeHorizon()
{
}

template <typename SCALAR>
bool MpcTimeHorizon<SCALAR>::computeNewTimeHorizon(const SCALAR& t_since_ended_first_solve,
    const SCALAR& t_forward_prediction_stop,
    SCALAR& new_T)
{
    /**
	 * compute desired end time and the time we have left from now. Will be ignored below, if not required in the scenario.
	 */
    SCALAR timeLeft = initialTimeHorizon_ - (t_since_ended_first_solve + t_forward_prediction_stop);
    timeLeft = std::max((SCALAR)0.0, timeLeft);


    switch (mpc_settings_.mpc_mode)
    {
        case MPC_MODE::CONSTANT_RECEDING_HORIZON:
        {
            new_T = initialTimeHorizon_;  // leave time horizon unchanged

            return false;  // in this mode, we never reach to the final time
        }
        case MPC_MODE::FIXED_FINAL_TIME:
        {
            new_T = timeLeft;

            if (new_T == 0.0)
                return true;  // reached time horizon, return true

            return false;
        }
        case MPC_MODE::FIXED_FINAL_TIME_WITH_MIN_TIME_HORIZON:
        {
            // std::max() ensures that the time is greater than the mininmum specified time horizon
            new_T = std::max((SCALAR)mpc_settings_.minimumTimeHorizonMpc_, timeLeft);

            if (new_T == mpc_settings_.minimumTimeHorizonMpc_)
                return true;

            return false;
        }
        case MPC_MODE::RECEDING_HORIZON_WITH_FIXED_FINAL_TIME:
        {
            new_T = std::min((SCALAR)mpc_settings_.minimumTimeHorizonMpc_, timeLeft);

            if (new_T == 0.0)
                return true;

            return false;
        }
        default:
            throw std::runtime_error("ERROR in MPC Constructor -- unknown Time Horizon Strategy.");
    }
}

template <typename SCALAR>
void MpcTimeHorizon<SCALAR>::updateSettings(const mpc_settings& mpcsettings)
{
    mpc_settings_ = mpcsettings;
}

template <typename SCALAR>
void MpcTimeHorizon<SCALAR>::updateInitialTimeHorizon(const SCALAR& initTimeHorizon)
{
    initialTimeHorizon_ = initTimeHorizon;
}

}  // namespace tpl
}  // namespace optcon
}  // namespace ct
