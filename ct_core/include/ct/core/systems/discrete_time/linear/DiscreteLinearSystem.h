/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! interface class for a general discrete linear system or linearized discrete system
/*!
 * Defines the interface for a discrete linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteLinearSystem : public DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
	 * @param type system type
	 */
    DiscreteLinearSystem(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type)
    {
    }

    //! destructor
    virtual ~DiscreteLinearSystem(){};

    //! deep cloning
    virtual DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override = 0;

    //! computes the controllability gramian by discretization
    /*!
     * Computes the controllability gramian
     *
     * @return Returns the controllability Gramian
     */
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> computeControllabilityGramian(const size_t max_iters=100,
        const double tolerance=1e-9)
    {
        state_matrix_t A;
        state_control_matrix_t B;

        // TODO: Need to get the matrices A and B
        //       Not neccessary to specifiy x and u?
        // this->getAandB(x, u, n, A, B);

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> CG_prev = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> CG      = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_prev  = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        size_t n_iters = 0;
        while (n_iters < max_iters)
        {
            CG = CG_prev + A_prev * B * B.transpose() * A_prev.transpose();

            // check for convergence using matrix 1-norm
            double norm1 = (CG_prev - CG).lpNorm<1>();
            if (norm1 < tolerance) {
                break;
            }

            // update
            A_prev = A_prev * A;
            CG_prev = CG;
            ++n_iters;
        }
        return CG;
    }

    //! compute the system dynamics
    /*!
	 * This computes the system dynamics
	 * \f[
	 *  x_{n+1} = Ax_n + Bu_n
	 * \f]
	 * @param state current state
	 * @param n current time index
	 * @param control control input
	 * @param stateNext propagated state
	 */
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        state_matrix_t A;
        state_control_matrix_t B;
        this->getAandB(state, control, state, n, 1, A, B);
        stateNext = A * state + B * control;
    }


    //! retrieve discrete-time linear system matrices A and B.
    /*!
     * This computes matrices A and B such that
     * \f[
     *  x_{n+1} = Ax_n + Bu_n
     * \f]
     *
     * Note that the inputs x_next and subSteps are potentially being ignored
     * for 'true' discrete system models but are relevant for sensitivity
     * calculations if the underlying system is continuous.
     *
     * @param x the state setpoint at n
     * @param u the control setpoint at n
     * @param n the time setpoint
     * @param x_next the state at n+1
     * @param subSteps number of substeps to use in sensitivity calculation
     * @param A the resulting linear system matrix A
     * @param B the resulting linear system matrix B
     */
    virtual void getAandB(const state_vector_t& x,
        const control_vector_t& u,
        const state_vector_t& x_next,
        const int n,
        size_t subSteps,
        state_matrix_t& A,
        state_control_matrix_t& B) = 0;

    void getAandB(const state_vector_t& x,
        const control_vector_t& u,
        const int n,
        state_matrix_t& A,
        state_control_matrix_t& B)
    {
        getAandB(x, u, x, n, 1, A, B);
    }
};

}  // namespace core
}  // namespace ct
