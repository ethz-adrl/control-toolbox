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

#ifdef USE_MATLAB_CPP_INTERFACE
#include <matlabCppInterface/Engine.hpp>
#endif

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
bool LQR<STATE_DIM, CONTROL_DIM>::compute(const state_matrix_t& Q,
	const control_matrix_t& R,
	const state_matrix_t& A,
	const control_gain_matrix_t& B,
	control_feedback_t& K,
	bool RisDiagonal,
	bool solveRiccatiIteratively)
{
	control_matrix_t R_inverse;
	state_matrix_t P;

	bool success = care_.solve(Q, R, A, B, P, RisDiagonal, R_inverse, solveRiccatiIteratively);

	K = (R_inverse * (B.transpose() * P));

	return success;
}

#ifdef USE_MATLAB_CPP_INTERFACE
template <size_t STATE_DIM, size_t CONTROL_DIM>
bool LQR<STATE_DIM, CONTROL_DIM>::computeMatlab(const state_matrix_t& Q,
	const control_matrix_t& R,
	const state_matrix_t& A,
	const control_gain_matrix_t& B,
	control_feedback_t& K,
	bool checkControllability)
{
	if (!matlabEngine_.isInitialized())
		matlabEngine_.initialize();

	matlabEngine_.put("Q", Q);
	matlabEngine_.put("R", R);
	matlabEngine_.put("A", A);
	matlabEngine_.put("B", B);

	if (checkControllability)
	{
		matlabEngine_.executeCommand("Co=ctrb(A,B);");
		matlabEngine_.executeCommand("unco=length(A)-rank(Co);");
		int uncontrollableStates = 0;
		matlabEngine_.get("unco", uncontrollableStates);

		if (uncontrollableStates > 0)
		{
			std::cout << "System is not controllable, # uncontrollable states: " << uncontrollableStates << std::endl;
		}
	}

	matlabEngine_.executeCommand("[K,~,~] = lqr(A,B,Q,R);");

	Eigen::MatrixXd Kmatlab;
	matlabEngine_.get("K", Kmatlab);

	K = Kmatlab;

	return true;
}
#endif  //USE_MATLAB_CPP_INTERFACE


}  // namespace optcon
}  // namespace ct
