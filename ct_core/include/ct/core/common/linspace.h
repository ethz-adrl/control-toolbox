/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
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
