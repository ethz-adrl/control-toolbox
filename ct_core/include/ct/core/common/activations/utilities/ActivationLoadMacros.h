/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define CT_LOADABLE_ACTIVATION(SCALAR, ACTIVATION, ACTIVATIONNAME)                                         \
    if (activationKind == ACTIVATIONNAME)                                                                  \
    {                                                                                                      \
        c_i = std::shared_ptr<ct::core::tpl::ACTIVATION<SCALAR>>(new ct::core::tpl::ACTIVATION<SCALAR>()); \
    }
