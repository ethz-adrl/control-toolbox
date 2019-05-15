/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

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
