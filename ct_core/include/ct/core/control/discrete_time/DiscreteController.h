/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! Interface class for all controllers
/*!
 * This is a pure interface class for Controllers that can be fed to any
 * ControlledSystem. Any custom controller should derive from this class
 * to ensure it is compatible with ControlledSystem and the Integrator.
 */
template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR = typename MANIFOLD::Scalar>
class DiscreteController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_t;

    DiscreteController() = default;
    virtual ~DiscreteController() = default;

    //! Copy constructor
    DiscreteController(const DiscreteController& other){};

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
    virtual void computeControl(const MANIFOLD& state, const int n, control_vector_t& controlAction) = 0;
};

}  // namespace core
}  // namespace ct
