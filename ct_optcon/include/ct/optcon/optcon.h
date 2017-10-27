/**********************************************************************************************************************
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
