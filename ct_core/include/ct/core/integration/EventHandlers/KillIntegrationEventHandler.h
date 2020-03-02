/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/integration/EventHandler.h>

namespace ct {
namespace core {

//! Event handler to kill integration
/*!
 * This event handler kills the integration if the kill flag is set. This is useful
 * for multi-threaded applications where an external kill signal needs to interrupt
 * an ongoing integration
 *
 * @tparam STATE_DIM size of the state vector
 */
template <typename MANIFOLD>
class KillIntegrationEventHandler : public EventHandler<MANIFOLD>
{
public:
    KillIntegrationEventHandler();

    //! default destructor
    virtual ~KillIntegrationEventHandler();

    virtual bool callOnSubsteps() override;

    //! checks the kill flag
    bool checkEvent(const MANIFOLD& state, const double& t) override;

    //! interrupts integration
    /*!
	 * interrupts the integration by throwing a std::runtime_error
	 * @param state current state (ignored)
	 * @param t current time (ignored)
	 */
    void handleEvent(const MANIFOLD& state, const double& t) override;

    //! enables killing at next call
    void setEvent();

    //! disable killing at next call
    void resetEvent();

    //! resets kill flag to false
    virtual void reset() override;

private:
    bool killIntegration_;  //! kill flag
};

}  // namespace core
}  // namespace ct
