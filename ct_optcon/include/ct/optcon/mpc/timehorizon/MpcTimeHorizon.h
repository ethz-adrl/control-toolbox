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

#ifndef CT_MPC_TIMEHORIZON_H_
#define CT_MPC_TIMEHORIZON_H_

#include <ct/core/core.h>
#include "../MpcSettings.h"

namespace ct{
namespace optcon{
namespace tpl{

/*!
 * This class implements the four default strategies for the time horizon in ct's MPC.
 * In case a different time horizon strategy is required, the user can derive from this
 * class and override the virtual functions
 */
template <typename SCALAR = double>
class MpcTimeHorizon
{

public:

	MpcTimeHorizon(const mpc_settings& settings, const SCALAR& initialTimeHorizon):
		mpc_settings_(settings),
		initialTimeHorizon_(initialTimeHorizon)
	{}


	virtual ~MpcTimeHorizon(){}


	//! compute new MPC time horizon
	/*!
	 * @param t_since_ended_first_solve
	 * 	Time since the first successful solve
	 * @param t_forward_prediction_stop
	 *  Relative time where to stop forward integration w.r.t. previous controller
	 * @param new_T
	 *  Resulting, new time horizon provided to the solver
	 * @return true if TimeHorizon reached and MPC should stop
	 */
	virtual bool computeNewTimeHorizon(
			const SCALAR& t_since_ended_first_solve,
			const SCALAR& t_forward_prediction_stop,
			SCALAR& new_T)
	{

		/**
		 * compute desired end time and the time we have left from now. Will be ignored below, if not required in the scenario.
		 */
		SCALAR timeLeft = initialTimeHorizon_ - (t_since_ended_first_solve + t_forward_prediction_stop);
		timeLeft = std::max((SCALAR)0.0, timeLeft);


		switch(mpc_settings_.mpc_mode)
		{
		case MPC_MODE::CONSTANT_RECEDING_HORIZON: {

			new_T = initialTimeHorizon_; // leave time horizon unchanged

			return false; // in this mode, we never reach to the final time
		}
		case MPC_MODE::FIXED_FINAL_TIME: {

			new_T = timeLeft;

			if(new_T == 0.0)
				return true; // reached time horizon, return true

			return false;
		}
		case MPC_MODE::FIXED_FINAL_TIME_WITH_MIN_TIME_HORIZON: {

			// std::max() ensures that the time is greater than the mininmum specified time horizon
			new_T = std::max((SCALAR)mpc_settings_.minimumTimeHorizonMpc_, timeLeft);

			if(new_T == mpc_settings_.minimumTimeHorizonMpc_)
				return true;

			return false;
		}
		case MPC_MODE::RECEDING_HORIZON_WITH_FIXED_FINAL_TIME: {

			new_T = std::min((SCALAR)mpc_settings_.minimumTimeHorizonMpc_, timeLeft);

			if(new_T == 0.0)
				return true;

			return false;
		}
		default:
			throw std::runtime_error("ERROR in MPC Constructor -- unknown Time Horizon Strategy.");
		}
	}


	void updateSettings(const mpc_settings& mpcsettings){ mpc_settings_ = mpcsettings;}


	//! update the time horizon which is used during the first call to the solver
	void updateInitialTimeHorizon(const SCALAR& initTimeHorizon){ initialTimeHorizon_ = initTimeHorizon;}


protected:

	mpc_settings mpc_settings_;

	SCALAR initialTimeHorizon_;

};

} // namespace tpl

typedef tpl::MpcTimeHorizon<double> MpcTimeHorizon;

} // namespace optcon
} // namespace ct




#endif /* CT_MPC_TIMEHORIZON_H_ */
