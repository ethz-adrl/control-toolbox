/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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

    typedef typename Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;            //!< state Jacobian type
    typedef typename Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t;  //!< input Jacobian type

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
    virtual LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const = 0;

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
    virtual void computeControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& derivative) override
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
    virtual const state_matrix_t& getDerivativeState(const StateVector<STATE_DIM, SCALAR>& x,const ControlVector<CONTROL_DIM, SCALAR>& u,const SCALAR t = 0.0) = 0;

    //! get the B matrix of a linear system
    /*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return B matrix
	 */
    virtual const state_control_matrix_t& getDerivativeControl(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const SCALAR t = 0.0) = 0;
};
}
}
