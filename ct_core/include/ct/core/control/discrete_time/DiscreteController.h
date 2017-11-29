/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! Interface class for all controllers
/*!
 * This is a pure interface class for Controllers that can be fed to any
 * ControlledSystem. Any custom controller should derive from this class
 * to ensure it is compatible with ControlledSystem and the Integrator.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! Default constructor
    DiscreteController(){};

    //! Copy constructor
    DiscreteController(const DiscreteController& other){};

    //! Destructor
    virtual ~DiscreteController(){};

    //! Deep cloning
    /*!
	 * Has to be implemented by any custom controller.
	 */
    virtual DiscreteController* clone() const = 0;

    //! Compute control signal
    /*!
	 * Evaluate the given controller for a given state and time index
	 * returns the computed control action.
	 *
	 * This function has to be implemented by any custom controller
	 *
	 * @param state current state of the system
	 * @param n current time index of the system
	 * @param controlAction the corresponding control action
	 */
    virtual void computeControl(const StateVector<STATE_DIM, SCALAR>& state,
        const int& n,
        ControlVector<CONTROL_DIM, SCALAR>& controlAction) = 0;
};

}  // namespace core
}  // namespace ct
