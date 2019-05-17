/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>

// Schur reordering from Lapack
#ifdef CT_USE_LAPACK
extern "C" void dtrsen_(const char* JOB,
    const char* COMPQ,
    const int* SELECT,
    const int* N,
    const double* T,
    const int* LDT,
    const double* Q,
    const int* LDQ,
    double* WR,
    double* WI,
    int* M,
    double* S,
    double* SEP,
    double* WORK,
    const int* LWORK,
    int* IWORK,
    const int* LIWORK,
    int* INFO);
#endif

namespace ct {
namespace optcon {

/*!
 * \ingroup LQR
 *+
 * \brief Continuous-Time Algebraic Riccati Equation
 *
 * solves the continuous-time Infinite-Horizon Algebraic Riccati Equation
 *
 * @tparam STATE_DIM system state dimension
 * @tparam CONTROL_DIM system control input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class CARE
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;

    typedef Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> schur_matrix_t;
    typedef Eigen::Matrix<double, 2 * STATE_DIM, STATE_DIM> factor_matrix_t;

    CARE();

    // Implementation using the Schur-Method
    // This is numerically more stable and should be preferred over the naive implementation
    bool solve(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        state_matrix_t& P,
        bool RisDiagonal,
        control_matrix_t& R_inverse,
        bool useIterativeSolver = false);

    state_matrix_t computeSteadyStateRiccatiMatrix(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        const bool RisDiagonal = false,
        const bool useIterativeSolver = false);

private:
    bool solveSchurIterative(const schur_matrix_t& M,
        state_matrix_t& P,
        double epsilon = 1e-6,
        int maxIterations = 100000);

    bool solveSchurDirect(const schur_matrix_t& M, state_matrix_t& P);

    Eigen::RealSchur<schur_matrix_t> schur_;
    Eigen::FullPivLU<factor_matrix_t> FullPivLU_;

    // Lapack
    int LWORK_;
    int LIWORK_;

    Eigen::VectorXd WORK_;
    Eigen::VectorXi IWORK_;
};
}  // namespace optcon
}  // namespace ct
