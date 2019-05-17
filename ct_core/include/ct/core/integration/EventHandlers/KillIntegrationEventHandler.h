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
template <size_t STATE_DIM>
class KillIntegrationEventHandler : public EventHandler<STATE_DIM>
{
public:
    typedef Eigen::Matrix<double, STATE_DIM, 1> State_T;

    //! default constructor
    /*!
	 * sets kill event to false
	 */
    KillIntegrationEventHandler() : killIntegration_(false) {}
    //! default destructor
    virtual ~KillIntegrationEventHandler() {}
    virtual bool callOnSubsteps() override { return false; }
    //! checks the kill flag
    bool checkEvent(const State_T& state, const double& t) override { return killIntegration_; }
    //! interrupts integration
    /*!
	 * interrupts the integration by throwing a std::runtime_error
	 * @param state current state (ignored)
	 * @param t current time (ignored)
	 */
    void handleEvent(const State_T& state, const double& t) override
    {
        /* throw an exception which stops the integration */
        throw std::runtime_error("Integration terminated due to external event specified by user.");
    }

    //! enables killing at next call
    void setEvent() { killIntegration_ = true; }
    //! disable killing at next call
    void resetEvent() { killIntegration_ = false; }
    //! resets kill flag to false
    virtual void reset() override { resetEvent(); };
private:
    bool killIntegration_;  //! kill flag
};

}  // namespace core
}  // namespace ct
