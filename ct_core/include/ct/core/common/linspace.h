/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/TypeTraits.h>
#include <manif/manif.h>

namespace ct {
namespace core {

//! replicates the well-known linspace command from MATLAB in C++
/*!
 * linspace provides exactly the same properties and functionality like in MATLAB.
 *
 * - N denotes the number of points, so you will obtain N-1 intervals
 * - a is the start of the interval
 * - b is the end of the interval
 *
 * Unit test \ref LinspaceTest.cpp illustrates the use of linspace.
 *
 */
template <typename ARRAY_T>
ARRAY_T linspace(const typename ARRAY_T::value_type& a, const typename ARRAY_T::value_type& b, const size_t N)
{
    if (N < 1)
        throw std::runtime_error("ERROR in CT_LINSPACE: N<1.");

    auto h = (b - a) / (N - 1);
    ARRAY_T traj(N);

    typename ARRAY_T::iterator it;
    typename ARRAY_T::value_type val;

    for (it = traj.begin(), val = a; it != traj.end(); ++it, val = val + h)
        *it = val;

    return traj;
}

}  // namespace core
}  // namespace ct
