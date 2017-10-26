/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include "utilities/TimeActivationLoadMacros.h"

#include "PeriodicActivation.hpp"
#include "SingleActivation.hpp"
#include "RBFGaussActivation.h"

#define CT_LOADABLE_TIME_ACTIVATIONS(SCALAR)                            \
    CT_LOADABLE_TIME_ACTIVATION(SCALAR, SingleActivation, "single")     \
    CT_LOADABLE_TIME_ACTIVATION(SCALAR, PeriodicActivation, "periodic") \
    CT_LOADABLE_TIME_ACTIVATION(SCALAR, RBFGaussActivation, "rbf")
