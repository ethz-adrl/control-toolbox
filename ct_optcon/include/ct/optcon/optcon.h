/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_OPTCON_OPTCON_H_
#define INCLUDE_CT_OPTCON_OPTCON_H_

#include <ct/core/core.h>

#include "costfunction/costfunction.hpp"

#include "constraint/constraint.h"

#include "problem/OptConProblem.h"
#include "problem/LQOCProblem.hpp"
#include "solver/OptConSolver.h"

#include "nloc/NLOCBackendBase.hpp"
#include "nloc/NLOCBackendST.hpp"
#include "nloc/NLOCBackendMP.hpp"
#include "nloc/algorithms/gnms/GNMS.hpp"
#include "nloc/algorithms/ilqr/iLQR.hpp"

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


// implementations
//costfunction
#include "costfunction/costfunction-impl.hpp"

//constraints
#include "constraint/constraint-impl.h"

//problem
#include "problem/OptConProblem-impl.h"
#include "problem/LQOCProblem-impl.hpp"

//solver
#include "solver/lqp/GNRiccatiSolver-impl.hpp"
#include "solver/lqp/HPIPMInterface-impl.hpp"
#include "solver/NLOptConSolver-impl.hpp"

//lqr
#include "lqr/riccati/CARE-impl.hpp"
#include "lqr/riccati/DARE-impl.hpp"
#include "lqr/FHDTLQR-impl.hpp"
#include "lqr/LQR-impl.hpp"

//nloc
#include "nloc/NLOCBackendBase-impl.hpp"
#include "nloc/NLOCBackendST-impl.hpp"
#include "nloc/NLOCBackendMP-impl.hpp"
#include "nloc/algorithms/gnms/GNMS-impl.hpp"
#include "nloc/algorithms/ilqr/iLQR-impl.hpp"

//mpc
#include "mpc/MPC-impl.h"
#include "mpc/timehorizon/MpcTimeHorizon-impl.h"
#include "mpc/policyhandler/PolicyHandler-impl.h"
#include "mpc/policyhandler/default/StateFeedbackPolicyHandler-impl.h"


// keep standard header guard (easy debugging)
// header guard is identical to the one in optcon-prespec.h
#endif /* INCLUDE_CT_OPTCON_OPTCON_H_ */
