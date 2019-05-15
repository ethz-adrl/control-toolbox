/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

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

// define namespace and base
#define ROBCOGEN_NS ct_InvertedPendulum
#define TARGET_NS InvertedPendulum

// define the links
#define CT_BASE fr_InvertedPendulumBase
#define CT_L0 fr_Link1

// define single end effector (could also be multiple)
#define CT_N_EE 1
#define CT_EE0 fr_ee
#define CT_EE0_IS_ON_LINK 1
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 0

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
#include "InvertedPendulumJointLimits.h"
#include "InvertedPendulumUrdfNames.h"
