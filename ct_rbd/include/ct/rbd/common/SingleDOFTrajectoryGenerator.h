/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "ct/core/core.h"

namespace ct {
namespace rbd {
namespace tpl {

/*!
 * Generate a simple trajectory for a single degree of freedom
 * 	 - the initial and target value
 * 	 - the max allowed acceleration and deceleration
 *
 * 	 This class comprises two different modes:
 *
 * 	 1) If the position distance to be bridged is too small to reach the maximum velocity through
 * 	 constant acceleration/deceleration, it simply constantly accelerates until the middle of the
 * 	 path and constantly decelerates after it.
 *
 * 	 2) Otherwise, it accelerates to the maximum velocity and then stays at that desired velocity.
 * 	 It starts decelerating just in time to reach zero velocity at the target point.
 *
 * */
template <typename SCALAR = double>
class SingleDOFTrajectoryGenerator
{
protected:
    enum MODE
    {
        ACCELERATE_AND_DECELERATE = 0,
        ACCELERATE_HOLD_DECELERATE = 1
    };

public:
    SingleDOFTrajectoryGenerator()
        : operation_mode_(ACCELERATE_AND_DECELERATE),
          x0_(SCALAR(0.0)),
          x_t0n_(SCALAR(0)),
          x_mid_(SCALAR(0)),
          x_t1_(SCALAR(0)),
          xT_(SCALAR(0.0)),
          v_max_(SCALAR(0.0)),
          a_max_(SCALAR(0.0)),
          t0_(SCALAR(0.0)),
          t0n_(SCALAR(0.0)),
          t1_(SCALAR(0.0)),
          T_(SCALAR(0.0))
    {
    }


    SingleDOFTrajectoryGenerator(const SCALAR x0, const SCALAR xT, SCALAR abs_v_max, SCALAR abs_a_max)
        : SingleDOFTrajectoryGenerator()
    {
        setup(x0, xT, abs_v_max, abs_a_max);
    }


    /*!
	 * @param x0 starting value for x
	 * @param xT end value for x
	 * @param abs_v_max maximal absolute velocity for the trajectory
	 * @param abs_a_max maximal absolute acceleration for the trajectory
	 *
	 * @warning this generator currently only supports zero start and end velocities
	 *
	 * @return the total trajectory time horizon computed from the given parameters.
	 * */
    SCALAR setup(const SCALAR x0, const SCALAR xT, SCALAR abs_v_max, SCALAR abs_a_max)
    {
        x0_ = x0;
        xT_ = xT;
        v_max_ = abs_v_max;
        a_max_ = abs_a_max;

        // check if we in fact need negative acceleration and velocities
        if (xT_ < x0_)
        {
            v_max_ = -v_max_;
            a_max_ = -a_max_;
        }

        // nominal time until the max velocity would be reached
        t0n_ = v_max_ / a_max_;
        // Distance which would be traveled during that time
        SCALAR delta_x = 0.5 * a_max_ * t0n_ * t0n_;

        // If the distance traveled would surpass the total position difference, we choose
        // ACCELERATE_AND_DECELERATE, otherwise we put a constant velocity hold segment in-between.
        if (2 * fabs(delta_x) >= fabs(xT_ - x0_))
        {
            operation_mode_ = ACCELERATE_AND_DECELERATE;

            SCALAR delta_x_mid = 0.5 * (xT_ - x0_);  // Half the distance between start and target
            x_mid_ = 0.5 * (xT_ + x0_);
            ;                                      // The point exactly in the middle
            t0_ = sqrt(delta_x_mid * 2 / a_max_);  // The time passed when driving there with const. acceleration
            v_max_ = a_max_ * t0_;                 // The velocity reached when arriving at x_mid - update v_max
            T_ = 2 * t0_;                          // The total move time horizon
        }
        else
        {
            operation_mode_ = ACCELERATE_HOLD_DECELERATE;

            x_t0n_ = x0_ + 0.5 * a_max_ * t0n_ * t0n_;  // The position reached after that time
            x_t1_ = xT_ - delta_x;  // The position where we start decelerating again after the constant vel. section
            t1_ = t0n_ + (x_t1_ - x_t0n_) / v_max_;  // The time when this happens, calculated from the const velocity.
            T_ = t1_ + t0n_;                         // The total move time horizon
        }

        return T_;
    }

    /*!
	 * @param time the trajectory query time
	 * @param x_new returns the evaluated value for position
	 * @param v_new returns the evaluated value for velocity
	 *
	 * @todo check and resolve why times would not be allowed to be negative.
	 * */
    void queryTrajectory(const SCALAR time, SCALAR& x_new, SCALAR& v_new)
    {
        if (time < SCALAR(0.0))
            throw std::runtime_error("SingleAxisTrajGenerator: time cannot be negative!");

        if (operation_mode_ == ACCELERATE_AND_DECELERATE)
            queryPlanAccADec(time, x_new, v_new);
        else if (operation_mode_ == ACCELERATE_HOLD_DECELERATE)
            queryPlanAccHDec(time, x_new, v_new);
        else
            throw std::runtime_error("SingleAxisTrajectoryGenerator: operation mode not available.");
    }


private:
    //! mode of operation for the trajectory generation
    MODE operation_mode_;
    SCALAR x0_, x_t0n_, x_mid_, x_t1_, xT_;
    SCALAR v_max_, a_max_;
    SCALAR t0_, t0n_, t1_, T_;

    void queryPlanAccADec(const SCALAR& time, SCALAR& x_new, SCALAR& v_new)
    {
        if (time <= t0_)
        {
            x_new = x0_ + 0.5 * a_max_ * time * time;  // Quadratic function for x
            v_new = a_max_ * time;                     // Linear function for vel
        }
        else if (time > t0_ && time <= 2 * t0_)
        {
            // Quadratic function for x
            x_new = x_mid_ + v_max_ * (time - t0_) - 0.5 * a_max_ * (time - t0_) * (time - t0_);
            // Linear function for vel
            v_new = v_max_ - a_max_ * (time - t0_);
        }
        else if (time > 2 * t0_)
        {
            x_new = xT_;  // Set to desired values if time horizon exceeded
            v_new = 0.0;
        }
        else
        {
            throw std::runtime_error("SingleAxisTrajectoryGenerator: something is wrong with the time.");
        }
    }

    void queryPlanAccHDec(const SCALAR& time, SCALAR& x_new, SCALAR& v_new)
    {
        if (time <= t0n_)
        {
            x_new = 0.5 * a_max_ * time * time + x0_;  // Quadratic function for x
            v_new = a_max_ * time;                     // Linear function for vel
        }
        else if (time > t0n_ && time <= t1_)
        {
            x_new = x_t0n_ + v_max_ * (time - t0n_);  // Position linearly increasing
            v_new = v_max_;                           // Constant velocity
        }
        else if (time > t1_ && time <= T_)
        {
            // x quadratically decreasing
            x_new = x_t1_ + v_max_ * (time - t1_) - 0.5 * a_max_ * ((time - t1_) * (time - t1_));
            // linearly decreasing
            v_new = v_max_ - ((time - t1_) * a_max_);
        }
        else if (time > T_)
        {
            x_new = xT_;  // Set to desired values if time horizon exceeded
            v_new = 0.0;
        }
        else
        {
            throw std::runtime_error("SingleAxisTrajectoryGenerator: something is wrong with the time.");
        }
    }
};


}  // tpl

using SingleDOFTrajectoryGenerator = tpl::SingleDOFTrajectoryGenerator<double>;

}  // rbd
}  // ct
