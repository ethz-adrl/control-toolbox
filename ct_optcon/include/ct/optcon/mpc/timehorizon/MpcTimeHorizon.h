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

#pragma once

#include "../MpcSettings.h"

namespace ct {
namespace optcon {
namespace tpl {

/*!
 * This class implements the four default strategies for the time horizon in ct's MPC.
 * In case a different time horizon strategy is required, the user can derive from this
 * class and override the virtual functions
 */
template <typename SCALAR = double>
class MpcTimeHorizon
{
public:
    MpcTimeHorizon(const mpc_settings& settings, const SCALAR& initialTimeHorizon);

    virtual ~MpcTimeHorizon();

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
    virtual bool computeNewTimeHorizon(const SCALAR& t_since_ended_first_solve,
        const SCALAR& t_forward_prediction_stop,
        SCALAR& new_T);

    void updateSettings(const mpc_settings& mpcsettings);

    //! update the time horizon which is used during the first call to the solver
    void updateInitialTimeHorizon(const SCALAR& initTimeHorizon);


protected:
    mpc_settings mpc_settings_;

    SCALAR initialTimeHorizon_;
};

}  // namespace tpl

typedef tpl::MpcTimeHorizon<double> MpcTimeHorizon;

}  // namespace optcon
}  // namespace ct
