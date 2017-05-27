// Timer for the HPCSE I course
//
#ifndef HPCSE_TIMER_HPP
#define HPCSE_TIMER_HPP

#include <sys/time.h>

class timer {
public:
	timer() {
		start_time.tv_sec  = 0;
		start_time.tv_usec = 0;
		stop_time.tv_sec   = 0;
		stop_time.tv_usec  = 0;
	}

	inline void start() {
		gettimeofday(&start_time, NULL);
	}

	inline void stop() {
		gettimeofday(&stop_time, NULL);
	}

	double get_timing() const {
		return (stop_time.tv_sec - start_time.tv_sec) + (stop_time.tv_usec - start_time.tv_usec)*1e-6;
	}

private:
	struct timeval start_time, stop_time;
};

#endif //HPCSE_TIMER_HPP
