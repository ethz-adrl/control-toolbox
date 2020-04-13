/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../arrays/TimeArray.h"
#include "../arrays/DiscreteArray.h"

#include <ct/core/common/Interpolation.h>


namespace ct {
namespace core {

//! A discrete, timed trajectory with interpolation
/*!
 * This implements a trajectory based on discrete data points which can be evenly or unevenly
 * distributed in time with time stamps associated with each data point. A basic interpolation
 * strategy can evaluate the trajectory also in between data points.
 *
 * An example how to use different features of the discrete trajectory can be found in unit test \ref DiscreteTrajectoryTest.cpp
 *
 * \tparam MANIFOLD type of each point of the trajectory
 * \tparam Alloc allocator for trajectory points
 *
 * */
template <typename MANIFOLD, class Alloc = Eigen::aligned_allocator<MANIFOLD>>
class DiscreteTrajectory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Scalar_t = typename MANIFOLD::Scalar;
    using alloc_type = Alloc;
    using DiscreteArray_t = DiscreteArray<MANIFOLD, alloc_type>;
    using DiscreteTrajectory_t = DiscreteTrajectory<MANIFOLD, Alloc>;

    //! default constructor
    DiscreteTrajectory(const InterpolationType& type = ZOH) : time_(), data_(), interp_(type) {}
    //! constructor
    /*!
	 * Initializes a trajectory from time stamps and corresponding data points. Both arrays need to be of
	 * same length. Additionally, an interpolation strategy is chosen.
	 *
	 * @param time time stamps
	 * @param data data points
	 * @param type interpolation strategy
	 */
    DiscreteTrajectory(const tpl::TimeArray<Scalar_t>& time,
        const DiscreteArray<MANIFOLD, Alloc>& data,
        const InterpolationType& type = ZOH)
        : time_(time), data_(data), interp_(type)
    {
    }

    //! constructor for uniformly spaced trajectories
    /*!
	 * Special-case constructor which makes a uniformly spaced time-trajectory
	 * @param data the data points
	 * @param deltaT time spacing between trajectory points
	 * @param t0 time of first data point
	 * @param type interpolation time
	 */
    DiscreteTrajectory(const DiscreteArray<MANIFOLD, Alloc>& data,
        const Scalar_t& deltaT,
        const Scalar_t& t0,
        const InterpolationType& type = ZOH)
        : time_(), data_(data), interp_(type)
    {
        time_ = tpl::TimeArray<Scalar_t>(deltaT, data.size(), t0);
    }

    //! copy constructor
    DiscreteTrajectory(const DiscreteTrajectory<MANIFOLD, Alloc>& other)
        : time_(other.time_), data_(other.data_), interp_(other.interp_.getInterpolationType())
    {
    }

    //! extraction constructor
    /*!
	 * Construct Trajectory from a segment of an existing trajectory
	 * @param other existing trajectory
	 * @param startIndex index where the trajectory to be extracted starts
	 * @param endIndex index where the trajectory to be extracted ends
	 */
    DiscreteTrajectory(DiscreteTrajectory<MANIFOLD, Alloc>& other, const size_t startIndex, const size_t endIndex)
        : time_(), data_(), interp_(other.interp_.getInterpolationType())
    {
        tpl::TimeArray<Scalar_t> time_temp;
        DiscreteArray<MANIFOLD, Alloc> data_temp;

        for (size_t i = startIndex; i <= endIndex; i++)
        {
            time_temp.push_back(other.time_[i]);
            data_temp.push_back(other.data_[i]);
        }

        time_ = time_temp;
        data_ = data_temp;
    }

    //! Destructor
    virtual ~DiscreteTrajectory() = default;


    //! set the data array
    /*!
	 * @param data new data array
	 */
    void setData(const DiscreteArray<MANIFOLD, Alloc>& data) { data_ = data; }
    //! set the interpolation strategy
    /*!
	 * @param type new interpolation strategy
	 */
    void setInterpolationType(const InterpolationType& type) { interp_.changeInterpolationType(type); }
    //! set timestamps
    /*!
	 * @param time new time stamps
	 */
    void setTime(const tpl::TimeArray<Scalar_t>& time) { time_ = time; }
    //! shift the trajectory forward in time
    /*!
	 * This method shifts the trajectory \b forward in time by applying a \b negative
	 * offset to all time stamps.
	 * @param dt offset (negative offset will be applied to timestamps)
	 */
    void shiftTime(const Scalar_t& dt) { time_.addOffset(-dt); }
    //! evaluate the trajectory at a certain time
    /*!
	 * This evaluates the trajectory at a certain time. If the exact time is not
	 * stored, the value will be interpolated.
	 * @param time stamp at which to evaluate the trajectory
	 * @return trajectory value
	 */
    virtual MANIFOLD eval(const Scalar_t& t)
    {
        MANIFOLD result;
        this->interp_.interpolate(time_, data_, t, result);
        return result;
    }
    //! returns the size of the trajectory
    /*!
	 * @return size of trajectory
	 */
    size_t size() const
    {
        if (data_.size() != time_.size())
            throw std::runtime_error("DiscreteTrajectory: size inconsistent.");

        return this->data_.size();
    }

    //! get the first element
    MANIFOLD& front() { return data_.front(); }
    //! get the first element
    const MANIFOLD& front() const { return data_.front(); }
    //! get the last element
    MANIFOLD& back() { return data_.back(); }
    //! get the last element
    const MANIFOLD& back() const { return data_.back(); }
    //! access a certain index of the trajectory (does not interpolate)
    MANIFOLD& operator[](const size_t i) { return data_[i]; }
    //! access a certain index of the trajectory (does not interpolate)
    const MANIFOLD& operator[](const size_t i) const { return data_[i]; }
    //! get the time stamp of the first element
    const Scalar_t startTime() const { return time_.front(); }
    //! get the time stamp of the last element
    const Scalar_t finalTime() const { return time_.back(); }
    //! time duration of the trajectory
    const Scalar_t duration() const { return time_.back() - time_.front(); }
    //! Add a data and time point at the end
    /*!
	 * Add a data-point and a time-point to the end of the trajectory.
	 * @param data
	 * @param time
	 * @param timeIsAbsolute
	 * 		true: time value is absolute, a simple push_back is sufficient
	 * 		false: time value is relative to current end of time trajectory
	 */
    void push_back(const MANIFOLD& data, const Scalar_t& time, const bool timeIsAbsolute)
    {
        if (timeIsAbsolute)
            time_.push_back(time);
        else
            time_.push_back(time + time_.back());

        data_.push_back(data);
    }

    //! Remove the last data and time pair
    void pop_back()
    {
        time_.pop_back();
        data_.pop_back();
    }

    //! Erase front elements and optionally shift the trajectory in time
    /*!
	 * @param N		number of elements to erase from the front
	 * @param dt	(optional) shift the time trajectory about dt.
	 */
    void eraseFront(const size_t& N, const Scalar_t& dt = 0.0)
    {
        time_.eraseFront(N);
        data_.eraseFront(N);
        shiftTime(dt);
    }

    //! Clear the trajectory
    void clear()
    {
        data_.clear();
        time_.clear();
    }

    //! Swap two trajectories
    /*!
	 * \warning swapping does not exchange the interpolation types, it just affects the data
	 *
	 * @param other trajectory to swap with
	 */
    void swapData(DiscreteTrajectory& other)
    {
        time_.swap(other.time_);
        data_.swap(other.data_);
    }

    //! assignment operator
    DiscreteTrajectory& operator=(const DiscreteTrajectory& other)
    {
        if (this == &other)
            return *this;

        time_ = other.time_;
        data_ = other.data_;
        interp_.changeInterpolationType(other.interp_.getInterpolationType());

        return *this;
    }

    //! get the time stamp at a certain index
    const Scalar_t& getTimeFromIndex(const size_t& ind) const { return time_[ind]; }
    //! get the index associated with a certain time
    /*!
	 * If the exact time is not stored, the interpolation will find the corresponding index.
	 * @param t time to search for
	 * @return according index
	 */
    size_t getIndexFromTime(const Scalar_t& t) { return interp_.findIndex(time_, t); }
    //! get the data array
    DiscreteArray<MANIFOLD, Alloc>& getDataArray() { return data_; }
    //! get the data array
    const DiscreteArray<MANIFOLD, Alloc>& getDataArray() const { return data_; }
    //! get the time array
    tpl::TimeArray<Scalar_t>& getTimeArray() { return time_; }
    //! get the time array
    const tpl::TimeArray<Scalar_t>& getTimeArray() const { return time_; }
    //! print out the trajectory
    /*!
	 * This is a convenience method allowing to quickly print out a trajectory, e.g. for debugging or for tutorial purposes.
	 */
    void print()
    {
        assert(time_.size() == data_.size());
        for (size_t i = 0; i < time_.size(); i++)
        {
            std::cout << "time: \t " << time_[i] << std::endl << "data-point: \t" << data_[i] << std::endl;
        }
    }

protected:
    tpl::TimeArray<Scalar_t> time_;                    //!< time array
    DiscreteArray<MANIFOLD, Alloc> data_;              //!< data array
    Interpolation<MANIFOLD, Alloc, Scalar_t> interp_;  //!< interpolation strategy
};

} /* namespace core */
} /* namespace ct */
