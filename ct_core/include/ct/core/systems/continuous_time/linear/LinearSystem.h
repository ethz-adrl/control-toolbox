/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! interface class for a general linear system or linearized system
/*!
 * Defines the interface for a linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class LinearSystem : public ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;

    typedef StateVector<STATE_DIM, SCALAR> state_vector_t;        //!< state vector type
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;  //!< input vector type

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
	 * @param type system type
	 */
    LinearSystem(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type)
    {
    }

    //! destructor
    virtual ~LinearSystem(){};

    //! deep cloning
    virtual LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override = 0;

    //! compute the system dynamics
    /*!
	 * This computes the system dynamics
	 * \f[
	 *  \dot{x} = Ax + Bu
	 * \f]
	 * @param state current state
	 * @param t current time
	 * @param control control input
	 * @param derivative state derivative
	 */
    virtual void computeControlledDynamics(const state_vector_t& state,
        const time_t& t,
        const control_vector_t& control,
        state_vector_t& derivative) override
    {
        // x_dot(t) = A(x,u,t) * x(t) + B(x,u,t) * u(t)

        derivative = getDerivativeState(state, control, t) * state + getDerivativeControl(state, control, t) * control;
    }

    //! get the A matrix of a linear system
    /*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return A matrix
	 */
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const time_t t = time_t(0.0)) = 0;

    //! get the B matrix of a linear system
    /*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return B matrix
	 */
    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const time_t t = time_t(0.0)) = 0;


    /**
     * @brief Get both linear system matrices A and B in one call.
     * 
     * Motvation: For certain derived instances, it may be more efficient to compute A and B simulatenously. 
     * In that case, this method can be overloaded for maximum efficiency. As default case, it simply calls above
     * methods independently.
     * 
     * @param A matrix A of linear system
     * @param B matrix B of linear system
     * @param x the current state
     * @param u the current input
     * @param t the current time
     */
    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const state_vector_t& x,
        const control_vector_t& u,
        const time_t t = time_t(0.0))
    {
        A = getDerivativeState(x, u, t);
        B = getDerivativeControl(x, u, t);
    }
};

}  // namespace core
}  // namespace ct
