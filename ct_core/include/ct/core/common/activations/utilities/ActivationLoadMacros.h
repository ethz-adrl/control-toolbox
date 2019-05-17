/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define CT_LOADABLE_ACTIVATION(SCALAR, ACTIVATION, ACTIVATIONNAME)                                         \
    if (activationKind == ACTIVATIONNAME)                                                                  \
    {                                                                                                      \
        c_i = std::shared_ptr<ct::core::tpl::ACTIVATION<SCALAR>>(new ct::core::tpl::ACTIVATION<SCALAR>()); \
    }
