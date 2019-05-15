/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

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
