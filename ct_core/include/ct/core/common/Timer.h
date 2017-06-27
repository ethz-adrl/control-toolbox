/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef CT_CORE_TIMER_H_
#define CT_CORE_TIMER_H_

#include <sys/time.h>

namespace ct{
namespace core{
namespace tpl{

//! A timer ("stop watch") to record elapsed time based on the system clock
/*!
 * Keeps track of time in a stop watch fashion.
 */
template <typename SCALAR = double>
class Timer {
public:

	//! Default constructor
	Timer()
	{
		start_time.tv_sec  = 0;
		start_time.tv_usec = 0;
		stop_time.tv_sec   = 0;
		stop_time.tv_usec  = 0;
	}

	//! Trigger start.
	/*!
	 * Starts the time measurement.
	 * Can be re-triggered without calling stop(). Simply overrides the start timestamp.
	 */
	inline void start() {
		gettimeofday(&start_time, NULL);
	}

	//! Trigger stop
	/*!
	 * Stops the time measurement.
	 */
	inline void stop() {
			gettimeofday(&stop_time, NULL);
	}

	//! Get the elapsed time between calls to start() and stop()
	/*!
	 *
	 * @return time in seconds
	 */
	SCALAR getElapsedTime() const {
		return (stop_time.tv_sec - start_time.tv_sec) + (stop_time.tv_usec - start_time.tv_usec)*1e-6;
	}

	//! Resets the clock.
	/*!
	 * Not needed to be called after start()/stop() calls.
	 */
	void reset(){
		start_time.tv_sec  = 0;
		start_time.tv_usec = 0;
		stop_time.tv_sec   = 0;
		stop_time.tv_usec  = 0;
	}

private:
	struct timeval start_time; /*!< start time */
	struct timeval stop_time; /*!< stop time */
};

}

typedef tpl::Timer<double> Timer;

}
}


#endif // CT_CORE_TIMER_H_
