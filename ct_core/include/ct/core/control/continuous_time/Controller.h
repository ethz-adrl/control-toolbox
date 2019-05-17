/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <memory>

#include <ct/core/types/Time.h>
#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>
#include <ct/core/types/ControlMatrix.h>


namespace ct {
namespace core {

//! Interface class for all controllers
/*!
 * This is a pure interface class for Controllers that can be fed to any
 * ControlledSystem. Any custom controller should derive from this class
 * to ensure it is compatible with ControlledSystem and the Integrator.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class Controller
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! Default constructor
    Controller(){};

    //! Copy constructor
    Controller(const Controller& other){};

    //! Destructor
    virtual ~Controller(){};

    //! Deep cloning
    /*!
	 * Has to be implemented by any custom controller.
	 */
    virtual Controller* clone() const = 0;

    //! Compute control signal
    /*!
	 * Evaluate the given controller for a given state and time
	 * returns the computed control action.
	 *
	 * This function has to be implemented by any custom controller
	 *
	 * @param state current state of the system
	 * @param t current time of the system
	 * @param controlAction the corresponding control action
	 */
    virtual void computeControl(const StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        ControlVector<CONTROL_DIM, SCALAR>& controlAction) = 0;

    /**
     * @brief      Returns the the derivative of the control with respect to the
     *             initial control input u0
     *
     * @param[in]  state  The state at which the method will be evaluated
     * @param[in]  time   The time at which the method will be evaluated
     *
     * @return     The derivatives with respect to u0.
     */
    virtual ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeU0(const StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR time)
    {
        throw std::runtime_error("getDerivativeU0() not implemented for the current controller");
    }

    /**
     * @brief      Returns the the derivative of the control with respect to the
     *             final control input uF
     *
     * @param[in]  state  The state at which the method will be evaluated
     * @param[in]  time   The time at which the method will be evaluated
     *
     * @return     The derivatives with respect to uF.
     */
    virtual ControlMatrix<CONTROL_DIM, SCALAR> getDerivativeUf(const StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR time)
    {
        throw std::runtime_error("getDerivativeUf() not implemented for the current controller");
    }
};

}  // namespace core
}  // namespace ct
