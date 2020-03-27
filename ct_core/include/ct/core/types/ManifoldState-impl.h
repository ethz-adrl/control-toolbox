/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CT_USE_MANIF

namespace ct {
namespace core {

template <typename M, typename T>
ManifoldState<M, T>::ManifoldState()
{
}

template <typename M, typename T>
ManifoldState<M, T>::~ManifoldState()
{
}

} /* namespace core */
} /* namespace ct */

#endif  // CT_USE_MANIF
