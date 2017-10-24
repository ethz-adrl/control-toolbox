/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
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
    virtual DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const = 0;

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
    virtual void propagateControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const int& n,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& stateNext) override
    {
        state_matrix_t A;
        state_control_matrix_t B;
        this->getAandB(state, control, state, n, 1, A, B);
        stateNext = A * state + B * control;
    }


    /*!
	 * retrieve discrete-time linear system matrices A and B.
	 * @param x	the state setpoint
	 * @param u the control setpoint
	 * @param n the time setpoint
	 * @param numSteps number of timesteps for which to get the sensitivity for
	 * @param A the resulting linear system matrix A
	 * @param B the resulting linear system matrix B
	 */
    virtual void getAandB(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const StateVector<STATE_DIM, SCALAR>& x_next,
        const int n,
        size_t numSteps,
        state_matrix_t& A,
        state_control_matrix_t& B) = 0;

    void getAandB(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const int n,
        state_matrix_t& A,
        state_control_matrix_t& B)
    {
        getAandB(x, u, x, n, 1, A, B);
    }
};

}  // core
}  // ct
