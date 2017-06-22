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

#ifndef TEST_MODELS_ROBCOGENHYA_H_
#define TEST_MODELS_ROBCOGENHYA_H_

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "generated/declarations.h"
#include "generated/jsim.h"
#include "generated/jacobians.h"
#include "generated/traits.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"

#include <ct/rbd/rbd.h>

// define namespace and base
#define ROBCOGEN_NS ct_HyA
#define TARGET_NS HyA

// define the links
#define CT_BASE fr_HyABase
#define CT_L0 fr_Shoulder_AA
#define CT_L1 fr_Shoulder_FE
#define CT_L2 fr_Humerus_R
#define CT_L3 fr_Elbow_FE
#define CT_L4 fr_Wrist_R
#define CT_L5 fr_Wrist_FE

// define single end effector (could also be multiple)
#define CT_N_EE 1
#define CT_EE0 fr_ee
#define CT_EE0_IS_ON_LINK 5
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 5

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
#include <ct/models/HyA/codegen/HyALinearizedForward.h>
#include <ct/models/HyA/codegen/HyAInverseDynJacReverse.h>

#include "HyAUrdfNames.h"

#endif /* ROBCOGENHYA_H_ */
