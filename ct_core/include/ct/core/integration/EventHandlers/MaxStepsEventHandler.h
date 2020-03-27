/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/integration/EventHandler.h>

namespace ct {
namespace core {

//! Event handler to kill a (variable step) Integrator after doing too many steps
/*!
 * Checks the number of steps that a variable step Integrator has taken and kills it if exceeded
 *
 * @tparam STATE_DIM state vector size
 */
template <typename MANIFOLD>
class MaxStepsEventHandler : public ct::core::EventHandler<MANIFOLD>
{
public:
    //! default constructor
    /*!
	 * @param maxStepsPerSec allowed number of steps
	 */
    MaxStepsEventHandler(const size_t& maxStepsPerSec = std::numeric_limits<size_t>::max());

    virtual ~MaxStepsEventHandler();

    virtual bool callOnSubsteps() override;

    //! resets the number of steps taken
    virtual void reset() override;

    //! checks if number of steps is exceeded
    /*!
	 *
	 * @param state current state (gets ignored)
	 * @param t current time (gets ignored)
	 * @return true if number of steps higher than maximum allowed number
	 */
    virtual bool checkEvent(const MANIFOLD& state, const double& t) override;

    //! throws a std::runtime_error to terminate the integration
    virtual void handleEvent(const MANIFOLD& state, const double& t) override;

    //! set maximum number of steps
    /*!
	 * @param maxNumSteps maximum number of steps allowed
	 */
    void setMaxNumSteps(size_t maxNumSteps);

private:
    size_t maxNumSteps_;  //! maximum number of steps allowed
    size_t stepsTaken_;   //! counter how many steps have passed
};

}  // namespace core
}  // namespace ct
