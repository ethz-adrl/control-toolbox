/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_OPTCON_OPTCON_H_
#define INCLUDE_CT_OPTCON_OPTCON_H_

#include <ct/core/core.h>

#include "costfunction/costfun.hpp"

#include "constraint/constraint.h"

#include "problem/OptConProblemBase.h"
#include "problem/ContinuousOptConProblem.h"
#include "problem/DiscreteOptConProblem.h"
#include "problem/LQOCProblem.hpp"
#include "solver/NLOptConSettings.hpp"

#include "system_interface/OptconSystemInterface.h"
#include "system_interface/OptconContinuousSystemInterface.h"
#include "system_interface/OptconDiscreteSystemInterface.h"

#include "nloc/NLOCBackendBase.hpp"
#include "nloc/NLOCBackendST.hpp"
#include "nloc/NLOCBackendMP.hpp"
#include "nloc/algorithms/MultipleShooting.hpp"
#include "nloc/algorithms/SingleShooting.hpp"

#include "solver/OptConSolver.h"
#include "solver/lqp/HPIPMInterface.hpp"
#include "solver/lqp/GNRiccatiSolver.hpp"
#include "solver/NLOptConSolver.hpp"

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

#include "filter/filter.h"

// implementations
#include "costfunction/costfun-impl.hpp"

#include "constraint/constraint-impl.h"

#include "system_interface/OptconContinuousSystemInterface-impl.h"
#include "system_interface/OptconDiscreteSystemInterface-impl.h"

#include "problem/OptConProblemBase-impl.h"
#include "problem/LQOCProblem-impl.hpp"

#include "solver/lqp/GNRiccatiSolver-impl.hpp"
#include "solver/lqp/HPIPMInterface-impl.hpp"
#include "solver/NLOptConSolver-impl.hpp"

#include "lqr/riccati/CARE-impl.hpp"
#include "lqr/riccati/DARE-impl.hpp"
#include "lqr/FHDTLQR-impl.hpp"
#include "lqr/LQR-impl.hpp"

#include "nloc/NLOCBackendBase-impl.hpp"
#include "nloc/NLOCBackendST-impl.hpp"
#include "nloc/NLOCBackendMP-impl.hpp"
#include "nloc/algorithms/MultipleShooting-impl.hpp"
#include "nloc/algorithms/SingleShooting-impl.hpp"

#include "mpc/MPC-impl.h"
#include "mpc/timehorizon/MpcTimeHorizon-impl.h"
#include "mpc/policyhandler/PolicyHandler-impl.h"
#include "mpc/policyhandler/default/StateFeedbackPolicyHandler-impl.h"

#include "filter/filter-impl.h"

// keep standard header guard (easy debugging)
// header guard is identical to the one in optcon-prespec.h
#endif /* INCLUDE_CT_OPTCON_OPTCON_H_ */
