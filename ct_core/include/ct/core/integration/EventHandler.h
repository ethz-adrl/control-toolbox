/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/arrays/DiscreteArray.h>
#include <ct/core/types/arrays/TimeArray.h>

namespace ct {
namespace core {

//! Interface for an event handler for an Integrator
/*!
 * An Integrator can call an EventHandler after each integration step to check or log
 * current states etc. This is useful for terminating integrators or checking for events
 * such as system dynamic switches in hybrid systems.
 *
 * Derive from this class to implement your custom event handler.
 *
 * @tparam STATE_DIM dimensionality of the state vector
 */
template <typename MANIFOLD, typename SCALAR = double>
class EventHandler
{
public:
    //! Default constructor
    EventHandler() = default;

    //! destructor
    virtual ~EventHandler() = default;

    virtual bool callOnSubsteps() = 0;

    //! reset event handler
    virtual void reset() = 0;

    //! check if an event has happened
    /*!
	 * checks if an event has happened and whether handleEvent() needs to be called
	 * @param state current state of the system
	 * @param t current time
	 * @return true if an event has happened
	 */
    virtual bool checkEvent(const MANIFOLD& state, const SCALAR& t) = 0;

    //! handle the event
    /*!
	 * does something with an event that just occurred
	 * @param state current state of the system
	 * @param t current time
	 */
    virtual void handleEvent(const MANIFOLD& state, const SCALAR& t) = 0;

private:
    ct::core::DiscreteArray<MANIFOLD> stateTrajectory_;  //! state trajectory for recording
    tpl::TimeArray<SCALAR> timeTrajectory_;              //! time trajectory for recording
};
}  // namespace core
}  // namespace ct
