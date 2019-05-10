/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EventHandler.h"

#include <ct/core/types/arrays/TimeArray.h>
#include <ct/core/types/arrays/MatrixArrays.h>

namespace ct {
namespace core {

template <size_t STATE_DIM, typename SCALAR>
class Integrator;

//! Observer for Integrator
/*!
 * Implements a general observer as required by boost::odeint. This wraps all event handlers and calls them.
 * Furthermore, this class records state and time trajectories during integration.
 *
 * @tparam STATE_DIM The size of the state vector
 */
template <size_t STATE_DIM, typename SCALAR = double>
class Observer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend class Integrator<STATE_DIM, SCALAR>;

    typedef std::vector<std::shared_ptr<EventHandler<STATE_DIM, SCALAR>>,
        Eigen::aligned_allocator<std::shared_ptr<EventHandler<STATE_DIM, SCALAR>>>>
        EventHandlerPtrVector;

    //! default constructor
    /*!
	 * @param eventHandlers vector of event handlers
	 */
    Observer(const EventHandlerPtrVector& eventHandlers);

    //! reset the observer
    void reset();

    void observe(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t);

    void log(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t);

    void observeInternal(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t);

private:
    //! Lambda to pass to odeint (odeint takes copies of the observer so we can't pass the class
    std::function<void(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)> observeWrap;
    std::function<void(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)> observeWrapWithLogging;

    ct::core::StateVectorArray<STATE_DIM, SCALAR> states_;  //!< container for logging the state
    ct::core::tpl::TimeArray<SCALAR> times_;                //!< container for logging the time


    EventHandlerPtrVector eventHandlers_;  //! list of event handlers
};
}
}
