/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

#include "generated/declarations.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/jacobians.h"
#include "generated/jsim.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"
#include "generated/traits.h"

#include <ct/rbd/rbd.h>


// these will be undefined later, DO NOT USE!

#define ROBCOGEN_NS TestHyQ  // defines the NS of the robot in robcogen, e.g. iit::<ROBCOGEN_NS>
#define TARGET_NS \
    TestHyQ  // defines the NS where all robot definitions go. Here ct::models::TestHyQ. This defines ct::models::TestHyQ::Dynamics etc.

// define all links of the robot, names as in robcogen
#define CT_BASE fr_trunk  // base link name
#define CT_L0 fr_LF_hipassembly
#define CT_L1 fr_LF_upperleg
#define CT_L2 fr_LF_lowerleg
#define CT_L3 fr_RF_hipassembly
#define CT_L4 fr_RF_upperleg
#define CT_L5 fr_RF_lowerleg
#define CT_L6 fr_LH_hipassembly
#define CT_L7 fr_LH_upperleg
#define CT_L8 fr_LH_lowerleg
#define CT_L9 fr_RH_hipassembly
#define CT_L10 fr_RH_upperleg
#define CT_L11 fr_RH_lowerleg

// number of endeffectors
#define CT_N_EE 4

// definition of an end-effector
#define CT_EE0 fr_LF_foot     //name of end-effector. Same as frame in RobCoGen
#define CT_EE0_IS_ON_LINK 3   // to which link is the end-effector rigidly attached to?
#define CT_EE0_FIRST_JOINT 0  // which is the first joint in the kinematic chain from base to end-effector?
#define CT_EE0_LAST_JOINT \
    2  // which is the last joint in the kinematic chain? THEY NEED TO BE IN ORDER, NO GAPS ALLOWED!

#define CT_EE1 fr_RF_foot
#define CT_EE1_IS_ON_LINK 6
#define CT_EE1_FIRST_JOINT 3
#define CT_EE1_LAST_JOINT 5

#define CT_EE2 fr_LH_foot
#define CT_EE2_IS_ON_LINK 9
#define CT_EE2_FIRST_JOINT 6
#define CT_EE2_LAST_JOINT 8

#define CT_EE3 fr_RH_foot
#define CT_EE3_IS_ON_LINK 12
#define CT_EE3_FIRST_JOINT 9
#define CT_EE3_LAST_JOINT 11

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
