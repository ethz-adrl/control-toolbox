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

#ifndef INCLUDE_CT_RBD_RBD_H_
#define INCLUDE_CT_RBD_RBD_H_

#include <ct/core/core.h>
#include <ct/rbd/internal/TraitSelectorSpecs.h>

#include <ct/rbd/common/SpatialForceVector.h>

#include <ct/rbd/state/RBDState.h>

#include <ct/rbd/robot/RobCoGenContainer.h>
#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/robot/Dynamics.h>

#include <ct/rbd/robot/control/IDControllerFB.h>
#include <ct/rbd/robot/control/WholeBodyController.h>
#include <ct/rbd/robot/control/InfiniteHorizonLQRwithInverseDynamics.h>

#include <ct/rbd/systems/FixBaseFDSystem.h>
#include <ct/rbd/systems/FloatingBaseFDSystem.h>
#include <ct/rbd/systems/ProjectedFDSystem.h>

#include <ct/rbd/slq/FloatingBaseSLQContactModel.h>
#include <ct/rbd/slq/FixBaseSLQ.h>


#endif /* INCLUDE_CT_RBD_RBD_H_ */
