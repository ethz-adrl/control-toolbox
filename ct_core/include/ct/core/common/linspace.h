/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

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
	 * */
template <typename TRAJECTORY_T>
TRAJECTORY_T linspace(const typename TRAJECTORY_T::value_type& a,
    const typename TRAJECTORY_T::value_type& b,
    const size_t N)
{
    if (N < 1)
        throw std::runtime_error("ERROR in CT_LINSPACE: N<1.");

    typename TRAJECTORY_T::value_type h = (b - a) / (N - 1);
    TRAJECTORY_T traj(N);

    typename TRAJECTORY_T::iterator it;
    typename TRAJECTORY_T::value_type val;

    for (it = traj.begin(), val = a; it != traj.end(); ++it, val += h)
        *it = val;

    return traj;
}

}  // namespace core
}  // namespace ct
