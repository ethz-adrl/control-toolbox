/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include "DynamicRiccatiEquation.hpp"

namespace ct {
namespace optcon {

/*!
 * \ingroup LQR
 *+
 * \brief Discrete-Time Algebraic Riccati Equation
 *
 * solves the discrete-time Infinite-Horizon Algebraic Riccati Equation iteratively
 *
 * @tparam STATE_DIM system state dimension
 * @tparam CONTROL_DIM system control input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DARE
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_feedback_t;

    DARE();

    /*! compute the discrete-time steady state Riccati-Matrix
     * this method iterates over the time-varying discrete-time Riccati Equation to compute the steady-state solution.
     * @param Q state weight
     * @param R control weight
     * @param A discrete-time linear system matrix A
     * @param B discrete-time linear system matrix B
     * @param verbose print additional information
     * @param eps treshold to stop iterating
     * @return steady state riccati matrix P
     */
    state_matrix_t computeSteadyStateRiccatiMatrix(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        control_feedback_t& K,
        bool verbose = false,
        const SCALAR eps = 1e-6,
        size_t maxIter = 1000);

    /*! compute the discrete-time steady state Riccati-Matrix with warm initialization of P matrix
     * this method iterates over the time-varying discrete-time Riccati Equation to compute the steady-state solution.
     * @param Q state weight
     * @param R control weight
     * @param A discrete-time linear system matrix A
     * @param B discrete-time linear system matrix B
     * @param P warm initialized P matrix
     * @param verbose print additional information
     * @param eps treshold to stop iterating
     * @return steady state riccati matrix P
     */
    state_matrix_t computeSteadyStateRiccatiMatrix(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        state_matrix_t P,
        control_feedback_t& K,
        bool verbose = false,
        const SCALAR eps = 1e-6,
        size_t maxIter = 1000);


private:
    DynamicRiccatiEquation<STATE_DIM, CONTROL_DIM> dynamicRDE_;
};

}  // namespace optcon
}  // namespace ct
