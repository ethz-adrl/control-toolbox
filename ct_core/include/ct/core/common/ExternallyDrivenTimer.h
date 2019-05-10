/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace tpl {

//! A timer ("stop watch") to record elapsed time based on external time stamps
/*!
 * Keeps track of time in a stop watch fashion.
 */
template <typename SCALAR = double>
class ExternallyDrivenTimer
{
public:
    //! Default constructor
    ExternallyDrivenTimer() { reset(); }
    //! Trigger start.
    /*!
	 * Starts the time measurement.
	 * Can be re-triggered without calling stop(). Simply overrides the start timestamp.
	 */
    inline void start(const SCALAR& time) { start_time = time; }
    //! Trigger stop
    /*!
	 * Stops the time measurement.
	 */
    inline void stop(const SCALAR& time) { stop_time = time; }
    //! Get the elapsed time between calls to start() and stop()
    /*!
	 * @return time
	 */
    SCALAR getElapsedTime() const { return stop_time - start_time; }
    //! Resets the clock.
    /*!
	 * Not needed to be called after start()/stop() calls.
	 */
    void reset()
    {
        start_time = (SCALAR)0.0;
        stop_time = (SCALAR)0.0;
    }

private:
    SCALAR start_time; /*!< start time */
    SCALAR stop_time;  /*!< stop time */
};
}

typedef tpl::ExternallyDrivenTimer<double> ExternallyDrivenTimer;
}
}
