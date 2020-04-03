/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CT_USE_MANIF

namespace ct {
namespace core {

template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
ManifoldState<MANIF_T, TAN, SCALAR>::ManifoldState()
{
}

template <template <class> class MANIF_T, template <class> class TAN, typename SCALAR>
ManifoldState<MANIF_T, TAN, SCALAR>::~ManifoldState()
{
}

} /* namespace core */
} /* namespace ct */

#endif  // CT_USE_MANIF
