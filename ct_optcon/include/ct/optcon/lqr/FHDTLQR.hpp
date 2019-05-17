/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/lqr/riccati/DynamicRiccatiEquation.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

namespace ct {
namespace optcon {

/*! \defgroup LQR LQR
 *
 * \brief Linear Quadratic Regulator Module
 * This module holds two verified LQR implementations in C++.
 */

/*!
 * \ingroup LQR
 *
 * \brief Finite-Horizon Discrete Time LQR
 *
 * compute the finite-horizon discrete time LQR solution
 * (Example: stabilize a linear time-varying system about a trajectory).
 * The user can either provide the linearized system matrices, or alternatively a pointer to a derivatives instance
 *
 * The feedback law has form
 * \f[
 * u_{fb} = -K \cdot (x - x_{ref})
 * \f]
 *
 * @tparam STATE_DIM system state dimension
 * @tparam CONTROL_DIM system input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class FHDTLQR
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_feedback_t;

    typedef core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;
    typedef core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;
    typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> state_matrix_array_t;

    typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> control_feedback_array_t;
    typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> control_gain_matrix_array_t;


    //! Constructor
    /*!
	 * @param costFunction the cost function to be used for designing the TVLQR
	 */
    FHDTLQR(std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFunction);

    ~FHDTLQR();


    //! design a time-varying feedback trajectory using user-provided matrices A and B
    /*!
	 *
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param A
	 * 	trajectory of system matrices A = df/dx
	 * @param B
	 *  trajectory of system matrices B = df/du
	 * @param dt
	 *  sampling time
	 * @param K
	 *  trajectory of resulting control feedback arrays
	 * @param performNumericalChecks
	 * (optional) perform some numerical checks while solving for K
	 */
    void designController(const state_vector_array_t& x_trajectory,
        const control_vector_array_t& u_trajectory,
        const state_matrix_array_t& A,
        const control_gain_matrix_array_t& B,
        SCALAR dt,
        control_feedback_array_t& K,
        bool performNumericalChecks = true);


    //! design a time-varying feedback trajectory using a user-provided derivative-pointer
    /*!
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param derivatives
	 *  shared_ptr to a LinearSystem, allowing to compute A and B
	 * @param dt
	 *  sampling time
	 * @param K
	 *  trajectory of resulting control feedback arrays
	 * @param performNumericalChecks
	 * (optional) perform some numerical checks while solving for K
	 */
    void designController(const state_vector_array_t& x_trajectory,
        const control_vector_array_t& u_trajectory,
        std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> derivatives,
        SCALAR dt,
        control_feedback_array_t& K,
        bool performNumericalChecks = true);


private:
    //! compute trajectories of A and B matrices along the given reference trajectory using the user-provided derivative instance
    /*!
	 *
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param N
	 *  number of discrete sampling points
	 * @param dt
	 *  sampling time dt
	 * @param derivatives
	 *  user-provided derivatives
	 * @param A
	 *  resulting linear state matrices A
	 * @param B
	 *  resulting linear state-input matrices B
	 */
    void linearizeModel(const state_vector_array_t& x_trajectory,
        const control_vector_array_t& u_trajectory,
        size_t N,
        SCALAR dt,
        std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& derivatives,
        state_matrix_array_t& A,
        control_gain_matrix_array_t& B);


    //! solve for the LQR feedback gains
    /*!
	 *
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param A
	 *  given linear state matrices A
	 * @param B
	 *  given linear state-input matrices B
	 * @param N
	 *  number of discrete sampling points
	 * @param dt
	 *  sampling time
	 * @param K
	 *  the resulting array of state-feedback matrices
	 * @param performNumericalChecks
	 *  (optional) perform some numerical checks while solving
	 */
    void solve(const state_vector_array_t& x_trajectory,
        const control_vector_array_t& u_trajectory,
        const state_matrix_array_t& A,
        const control_gain_matrix_array_t& B,
        size_t N,
        SCALAR dt,
        control_feedback_array_t& K,
        bool performNumericalChecks = true);


    std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>>
        costFunction_;  //! a quadratic costfunction for solving the optimal control problem
    DynamicRiccatiEquation<STATE_DIM, CONTROL_DIM, SCALAR> ricattiEq_;  //! the Riccati Equations
};

}  // namespace optcon
}  // namespace ct
