/*
 * TimeGrid.hpp
 *
 * Created: 15.01.2016
 * Author: mgiftthaler
 *
 *
 * This class describes the time-grid underlying the shots in the dms problem
 * In total, we have N+1 pairs of (s_i, q_i) and N shots between them.
 *
 * We assume that starting time t_0 = 0.0 [sec]
 *
 * */


#ifndef DMS_TIME_GRID_HPP_
#define DMS_TIME_GRID_HPP_

//#define DEBUG_TIMEGRID

#include <math.h>
#include <cmath>
#include <ct/core/core.h>

#include <ct/core/Types>

namespace ct {
namespace optcon {

class TimeGrid
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TimeGrid() = delete;

	TimeGrid(const size_t numberOfShots, const double timeHorizon):
	numberOfShots_(numberOfShots),
	timeHorizon_(timeHorizon),
	t_(numberOfShots + 1, 0.0)
	{
		makeUniformGrid();
	}

	void changeShotNr(const size_t numberOfShots)
	{
		numberOfShots_ = numberOfShots;
		t_.clear();
		t_.resize(numberOfShots_ + 1, 0.0);
		makeUniformGrid();
	}

	void changeTimeHorizon(const double timeHorizon)
	{
		timeHorizon_ = timeHorizon;
		makeUniformGrid();
	}


	/* Update the time grid. Expects shot durations h_0, h_1, ..., h_(N-1)
	 * Note: this function is supposed to be called ONLY by the OptVector class
	 * */
	void updateTimeGrid(const Eigen::VectorXd& h_segment)
	{
		t_[0] = 0.0; //by convention (just for documentation)

		for(size_t i = 0; i < (size_t) h_segment.size(); ++i)
		{
			t_[i+1] = t_[i] + h_segment(i);
			assert(t_[i+1] == t_[i+1]);
		}

#ifdef DEBUG_TIMEGRID
		std::cout << " ... in updateTimeGrid(). t_ =  ";
		for(size_t i = 0; i<t_.size(); i++)
			std::cout << std::setprecision (10) <<t_[i] << "  ";

		std::cout << std::endl;
#endif
	}


	/* function for uniformly distributing the shooting intervals subject to
	 * a guess for the overall movement duration T*/
	void makeUniformGrid()
	{
		for (size_t i = 0; i < numberOfShots_ + 1; i++)
		{
			t_[i]	= i * (double)(timeHorizon_/ (double)numberOfShots_);
		}
	}


	/**
	 * getShotIdx()
	 * 	 returns the index of a shot in which a certain time t lies.
	 * 	 the shot intervals are considered as closed on the left and open on the right: [t_start t_end[
	 *
	 * Notes on the logic:
	 * - in order to make sure that a time t which lies exactly at the matching
	 *   point between two shots gets assigned to the shot with higher index,
	 *   we add some epsilon to the inquried time t.
	 * - if the inquired time t is mapped to the last time in the grid, we make the
	 *   iterator point to the second-to-last time (iterator-2). (1)
	 * - if the inquired time t is greater than the starting time in t_.begin(),
	 *   std::upper_bound returns an iterator pointing to the NEXT element, thus
	 *   step back one element (2).
	 *   This also means: if the inquired time t is smaller than the starting time in t_.begin(),
	 * 	 std::upper_bound will return an iterator pointing to the starting time,
	 * 	 which corresponds to shot index 0.
	 *
	 * */
	const size_t getShotIdx(const ct::core::Time& t)
	{
		auto low = std::upper_bound (t_.begin(), t_.end(), t+10*std::numeric_limits<ct::core::Time>::epsilon());

		if(low == t_.end())
			low = t_.end()-2;  	// see (1)
		else if(low > t_.begin())
			--low;				// see (2)

#ifdef DEBUG_TIMEGRID
		std::cout << "time t: \t" << t << "\t t lower bound: \t"<< *low << "\t shot idx: \t" << (size_t) (low - t_.begin()) << std::endl;
#endif

		return (size_t) std::floor(low - t_.begin());
	}


	// get a shot's start time from its index
	const ct::core::Time& getShotStartTime(const size_t& shot_index){
		return t_[shot_index];
	}

	// get a shot's end time from its index
	const ct::core::Time& getShotEndTime(const size_t& shot_index){
		return t_[shot_index+1];
	}

	// get a shot's current duration from its index
	const ct::core::Time getShotDuration(const size_t& shot_index){
		return (t_[shot_index+1]-t_[shot_index]);
	}

	// return reference to the time grid vector
	ct::core::TimeArray& getTimeGrid()
	{
		return t_;
	}


	// return the total manoeuver time
	const ct::core::Time getTtotal()
	{
#ifdef DEBUG_TIMEGRID
		std::cout << "T-Horizon " << t_.back() << "  T at [N] "<< t_[N_] << std::endl;
#endif

		return t_.back();
	}


private:
	size_t numberOfShots_;
	double timeHorizon_;

	// the individual times of each pair from i=0,..., N
	ct::core::TimeArray t_;
};

} // namespace optcon
} // namespace ct


#endif
