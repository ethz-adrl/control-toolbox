/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <sys/time.h>

namespace ct {
namespace core {
namespace tpl {

//! A timer ("stop watch") to record elapsed time based on the system clock
/*!
 * Keeps track of time in a stop watch fashion.
 */
template <typename SCALAR = double>
class Timer
{
public:
    //! Default constructor
    Timer() { reset(); }
    //! Trigger start.
    /*!
	 * Starts the time measurement.
	 * Can be re-triggered without calling stop(). Simply overrides the start timestamp.
	 */
    inline void start() { gettimeofday(&start_time, NULL); }
    //! Trigger stop
    /*!
	 * Stops the time measurement.
	 */
    inline void stop() { gettimeofday(&stop_time, NULL); }
    //! Get the elapsed time between calls to start() and stop()
    /*!
	 *
	 * @return time in seconds
	 */
    SCALAR getElapsedTime() const
    {
        return (stop_time.tv_sec - start_time.tv_sec) + (stop_time.tv_usec - start_time.tv_usec) * 1e-6;
    }

    //! Resets the clock.
    /*!
	 * Not needed to be called after start()/stop() calls.
	 */
    void reset()
    {
        start_time.tv_sec = 0;
        start_time.tv_usec = 0;
        stop_time.tv_sec = 0;
        stop_time.tv_usec = 0;
    }

private:
    struct timeval start_time; /*!< start time */
    struct timeval stop_time;  /*!< stop time */
};
}

typedef tpl::Timer<double> Timer;
}
}
