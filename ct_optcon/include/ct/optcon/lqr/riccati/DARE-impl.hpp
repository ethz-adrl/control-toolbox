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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
DARE<STATE_DIM, CONTROL_DIM, SCALAR>::DARE()
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename DARE<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t DARE<STATE_DIM, CONTROL_DIM, SCALAR>::computeSteadyStateRiccatiMatrix(
		const state_matrix_t& Q,
		const control_matrix_t& R,
		const state_matrix_t& A,
		const control_gain_matrix_t& B,
		control_feedback_t& K,
		bool verbose,
		const SCALAR eps,
		size_t maxIter)
{
	state_matrix_t P, P_prev;
	size_t numIter = 0;

	SCALAR diff = 1;

	while (diff >= eps && numIter < maxIter)
	{
		state_matrix_t P_prev = P;
		dynamicRDE_.iterateRobust(Q, R, A, B, P, K);
		diff = (P-P_prev).cwiseAbs().maxCoeff();
		numIter++;
	}

	if(verbose)
		std::cout << "DARE : converged after " << numIter << " iterations out of a maximum of "<< maxIter << std::endl;

	return P;
}

}	// namespace optcon
}	// namespace ct

