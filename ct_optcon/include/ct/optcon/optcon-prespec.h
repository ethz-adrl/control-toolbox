/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_OPTCON_OPTCON_H_
#define INCLUDE_CT_OPTCON_OPTCON_H_

#include <ct/core/core-prespec.h>

#include "costfunction/costfunction.hpp"
#include "costfunction/costfunction-impl.hpp"  //temporary solution (todo)

#include "filter/filter.h"
#include "filter/filter-impl.h"  //temporary solution (todo)

#include "constraint/constraint.h"

#include "problem/OptConProblemBase.h"
#include "problem/ContinuousOptConProblem.h"
#include "problem/DiscreteOptConProblem.h"
#include "problem/LQOCProblem.hpp"

#include "system_interface/OptconSystemInterface.h"
#include "system_interface/OptconContinuousSystemInterface.h"
#include "system_interface/OptconDiscreteSystemInterface.h"

#include "nloc/NLOCBackendBase.hpp"
#include "nloc/NLOCBackendST.hpp"
#include "nloc/NLOCBackendMP.hpp"
#include "nloc/algorithms/gnms/GNMS.hpp"
#include "nloc/algorithms/ilqr/iLQR.hpp"

#include "solver/OptConSolver.h"
#include "solver/lqp/HPIPMInterface.hpp"
#include "solver/lqp/GNRiccatiSolver.hpp"
#include "solver/NLOptConSolver.hpp"
#include "solver/NLOptConSettings.hpp"

#include "lqr/riccati/CARE.hpp"
#include "lqr/riccati/DARE.hpp"
#include "lqr/FHDTLQR.hpp"
#include "lqr/LQR.hpp"

#include "dms/dms.h"

#include "mpc/MpcSettings.h"
#include "mpc/MPC.h"
#include "mpc/timehorizon/MpcTimeHorizon.h"
#include "mpc/policyhandler/PolicyHandler.h"
#include "mpc/policyhandler/default/StateFeedbackPolicyHandler.h"

/*!
 * \warning{do not include implementation files in optcon-prespec.h}
 */

// keep standard header guard (easy debugging)
// header guard is identical to the one in optcon.h
#endif /* INCLUDE_CT_OPTCON_OPTCON_H_ */
