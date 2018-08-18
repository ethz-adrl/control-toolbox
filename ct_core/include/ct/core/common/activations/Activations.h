/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * A convenience include which defines loadable time activations.
 */

#pragma once

#include "../activations/ActivationBase.hpp"
#include "../activations/PeriodicActivation.hpp"
#include "../activations/RBFGaussActivation.h"
#include "../activations/SingleActivation.hpp"
#include "../activations/BarrierActivation.hpp"
#include "../activations/LinearActivation.hpp"
#include "../activations/utilities/ActivationLoadMacros.h"

#define CT_LOADABLE_ACTIVATIONS(SCALAR)                            \
    CT_LOADABLE_ACTIVATION(SCALAR, SingleActivation, "single")     \
    CT_LOADABLE_ACTIVATION(SCALAR, PeriodicActivation, "periodic") \
    CT_LOADABLE_ACTIVATION(SCALAR, RBFGaussActivation, "rbf")      \
    CT_LOADABLE_ACTIVATION(SCALAR, BarrierActivation, "barrier")   \
    CT_LOADABLE_ACTIVATION(SCALAR, LinearActivation, "linear")
