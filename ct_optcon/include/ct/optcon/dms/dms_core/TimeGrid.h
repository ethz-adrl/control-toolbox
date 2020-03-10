/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

//#define DEBUG_TIMEGRID

#include <math.h>
#include <cmath>

namespace ct {
namespace optcon {
namespace tpl {

/**
 * @ingroup    DMS
 *
 * This class describes the time-grid underlying the shots in the dms problem In
 * total, we have N+1 pairs of (s_i, q_i) and N shots between them.
 *
 * We assume that starting time t_0 = 0.0 [sec]
 */
template <typename SCALAR>
class TimeGrid
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TimeGrid() = delete;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  numberOfShots  The number of shots
	 * @param[in]  timeHorizon    The dms time horizon
	 */
    TimeGrid(const size_t numberOfShots, const SCALAR timeHorizon)
        : numberOfShots_(numberOfShots), timeHorizon_(timeHorizon), t_(numberOfShots + 1, SCALAR(0.0))
    {
        makeUniformGrid();
    }

    /**
	 * @brief      Updates the timegrid when the number of shots changes
	 *
	 * @param[in]  numberOfShots  The new number of shots
	 */
    void changeShotCount(const size_t numberOfShots)
    {
        numberOfShots_ = numberOfShots;
        t_.clear();
        t_.resize(numberOfShots_ + 1, SCALAR(0.0));
        makeUniformGrid();
    }

    /**
	 * @brief      Updates the timegrid when the timehorizon changes
	 *
	 * @param[in]  timeHorizon  The new time horizon
	 */
    void changeTimeHorizon(const SCALAR timeHorizon)
    {
        timeHorizon_ = timeHorizon;
        makeUniformGrid();
    }


    /**
	 * @brief      This method updates the timegrid when new optimized time
	 *             segments arrive from the nlp solver. Only gets called when
	 *             using timegrid optimization, otherwise the timegrid stays
	 *             fixed
	 *
	 * @param[in]  h_segment  The vector of the new optimized time segments
	 */
    void updateTimeGrid(const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& h_segment)
    {
        t_[0] = SCALAR(0.0);  //by convention (just for documentation)

        for (size_t i = 0; i < (size_t)h_segment.size(); ++i)
            t_[i + 1] = t_[i] + h_segment(i);

#ifdef DEBUG_TIMEGRID
        std::cout << " ... in updateTimeGrid(). t_ =  ";
        for (size_t i = 0; i < t_.size(); i++)
            std::cout << std::setprecision(10) << t_[i] << "  ";

        std::cout << std::endl;
#endif
    }


    /**
	 * @brief      Creates a uniform timegrid
	 */
    void makeUniformGrid()
    {
        for (size_t i = 0; i < numberOfShots_ + 1; i++)
            t_[i] = i * (SCALAR)(timeHorizon_ / (SCALAR)numberOfShots_);
    }


    /**
	 * @brief      Returns to start time of a shot
	 *
	 * @param[in]  shot_index  The shot number
	 *
	 * @return     The start time
	 */
    const SCALAR getShotStartTime(const size_t shot_index) const { return t_[shot_index]; }
    /**
	 * @brief      Returns the end time of a shot
	 *
	 * @param[in]  shot_index  The shot number
	 *
	 * @return     The end time
	 */
    const SCALAR getShotEndTime(const size_t shot_index) const { return t_[shot_index + 1]; }
    /**
	 * @brief      Returns to duration of a shot
	 *
	 * @param[in]  shot_index  The shot index
	 *
	 * @return     The duration
	 */
    const SCALAR getShotDuration(const size_t shot_index) const { return (t_[shot_index + 1] - t_[shot_index]); }
    /**
	 * @brief      Returns the underlying TimeArray
	 *
	 * @return     The underlying TimeArray
	 */
    const ct::core::tpl::TimeArray<SCALAR>& toImplementation() { return t_; }
    /**
	 * @brief      Returns the initial timehorizon of the problem
	 *
	 * @return     The initial time horizon
	 */
    const SCALAR getTimeHorizon() const { return timeHorizon_; }
    /**
	 * @brief      Returns the optimized timehorizon
	 *
	 * @return     The optimized timehorizon
	 */
    const SCALAR getOptimizedTimeHorizon() const { return t_.back(); }
private:
    size_t numberOfShots_;
    SCALAR timeHorizon_;

    // the individual times of each pair from i=0,..., N
    ct::core::tpl::TimeArray<SCALAR> t_;
};
}

typedef tpl::TimeGrid<double> TimeGrid;

}  // namespace optcon
}  // namespace ct
