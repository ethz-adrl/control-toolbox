/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/rbd.h>

#include "generated/declarations.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/jacobians.h"
#include "generated/jsim.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"
#include "generated/traits.h"


// define namespace and base
#define ROBCOGEN_NS ct_quadrotor
#define TARGET_NS quadrotor

// define the links
#define CT_BASE fr_body
#define CT_L0 link1
#define CT_L1 link2

// define first end effector, the endeffector frame
#define CT_N_EE 1
#define CT_EE0 fr_ee
#define CT_EE0_IS_ON_LINK 1
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 1

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
