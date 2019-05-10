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
template <size_t STATE_DIM>
class MaxStepsEventHandler : public ct::core::EventHandler<STATE_DIM>
{
public:
    typedef ct::core::StateVector<STATE_DIM> State_T;

    //! default constructor
    /*!
	 * @param maxStepsPerSec allowed number of steps
	 */
    MaxStepsEventHandler(const size_t& maxStepsPerSec = std::numeric_limits<size_t>::max())
        : maxNumSteps_(maxStepsPerSec), stepsTaken_(0)
    {
    }

    //! destructor
    virtual ~MaxStepsEventHandler() {}
    virtual bool callOnSubsteps() override { return false; }
    //! resets the number of steps taken
    virtual void reset() override { stepsTaken_ = 0; };
    //! checks if number of steps is exceeded
    /*!
	 *
	 * @param state current state (gets ignored)
	 * @param t current time (gets ignored)
	 * @return true if number of steps higher than maximum allowed number
	 */
    virtual bool checkEvent(const State_T& state, const double& t) override
    {
        stepsTaken_++;
        return (stepsTaken_ > maxNumSteps_);  // todo: fix this
    }

    //! throws a std::runtime_error to terminate the integration
    virtual void handleEvent(const State_T& state, const double& t) override
    {
        throw std::runtime_error("integration terminated: max number of steps reached.\n");
    }

    //! set maximum number of steps
    /*!
	 * @param maxNumSteps maximum number of steps allowed
	 */
    void setMaxNumSteps(size_t maxNumSteps) { maxNumSteps_ = maxNumSteps; }
private:
    size_t maxNumSteps_;  //! maximum number of steps allowed
    size_t stepsTaken_;   //! counter how many steps have passed
};

}  // namespace core
}  // namespace ct
