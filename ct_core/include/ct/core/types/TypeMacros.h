/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TypeTraits.h"

#define CT_SYMPLECTIC_ENABLED(MANIFOLD) \
    template <class MF = MANIFOLD>           \
    typename std::enable_if<ct::core::is_symplectic<MF>::value, void>::type

#define CT_SYMPLECTIC_DISABLED(MANIFOLD) \
    template <class MF = MANIFOLD>            \
    typename std::enable_if<!(ct::core::is_symplectic<MF>::value), void>::type
