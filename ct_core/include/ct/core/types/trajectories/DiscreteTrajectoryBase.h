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

#ifndef CT_DISCRETETRAJECTORYBASE_H_
#define CT_DISCRETETRAJECTORYBASE_H_

#include "TrajectoryBase.h"
#include "TimeArray.h"
#include "DiscreteArray.h"

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
 * \tparam T type of each point of the trajectory
 * \tparam Alloc allocator for trajectory points
 */
template <class T, class Alloc = Eigen::aligned_allocator<T>, typename SCALAR = double>
class DiscreteTrajectoryBase : public TrajectoryBase<T, SCALAR>
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! default constructor
	DiscreteTrajectoryBase(const InterpolationType& type = ZOH):
		time_(),
		data_(),
		interp_(type)
	{}

	//! constructor
	/*!
	 * Initializes a trajectory from time stamps and corresponding data points. Both arrays need to be of
	 * same length. Additionally, an interpolation strategy is chosen.
	 *
	 * @param time time stamps
	 * @param data data points
	 * @param type interpolation strategy
	 */
	DiscreteTrajectoryBase(const tpl::TimeArray<SCALAR>& time, const DiscreteArray<T, Alloc>& data, const InterpolationType& type = ZOH):
		time_(time),
		data_(data),
		interp_(type)
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
	DiscreteTrajectoryBase(const DiscreteArray<T, Alloc>& data, const Time& deltaT, const Time& t0, const InterpolationType& type = ZOH):
			time_(),
			data_(data),
			interp_(type)
	{
		time_ = tpl::TimeArray<SCALAR>(deltaT, data.size(), t0);
	}

	//! copy constructor
	DiscreteTrajectoryBase(const DiscreteTrajectoryBase<T, Alloc>& other) :
		time_(other.time_),
		data_(other.data_),
		interp_(other.interp_.getInterpolationType())
	{
	}

	//! extraction constructor
	/*!
	 * Construct Trajectory from a segment of an existing trajectory
	 * @param other existing trajectory
	 * @param startIndex index where the trajectory to be extracted starts
	 * @param endIndex index where the trajectory to be extracted ends
	 */
	DiscreteTrajectoryBase(DiscreteTrajectoryBase<T, Alloc, SCALAR>& other, const size_t startIndex, const size_t endIndex):
		time_(),
		data_(),
		interp_(other.interp_.getInterpolationType())
	{
		tpl::TimeArray<SCALAR> time_temp;
		DiscreteArray<T, Alloc> data_temp;

		for(size_t i = startIndex; i<=endIndex; i++)
		{
			time_temp.push_back(other.time_[i]);
			data_temp.push_back(other.data_[i]);
		}

		time_ = time_temp;
		data_ = data_temp;
	}

	//! Destructor
	virtual ~DiscreteTrajectoryBase() {};


	//! set the data array
	/*!
	 * @param data new data array
	 */
	void setData(const DiscreteArray<T, Alloc>& data) {
		data_ = data;
	}

	//! set the interpolation strategy
	/*!
	 * @param type new interpolation strategy
	 */
	void setInterpolationType(const InterpolationType& type){
		interp_.changeInterpolationType(type);
	}

	//! set timestamps
	/*!
	 * @param time new time stamps
	 */
	void setTime(const tpl::TimeArray<SCALAR>& time) {
		time_ = time;
	}

	//! shift the trajectory forward in time
	/*!
	 * This method shifts the trajectory \b forward in time by applying a \b negative
	 * offset to all time stamps.
	 * @param dt offset (negative offset will be applied to timestamps)
	 */
	void shiftTime(const Time& dt) {
		time_.addOffset(-dt);
	}

	//! evaluate the trajectory at a certain time
	/*!
	 * This evaluates the trajectory at a certain time. If the exact time is not
	 * stored, the value will be interpolated.
	 * @param time stamp at which to evaluate the trajectory
	 * @return trajectory value
	 */
	virtual T eval(const SCALAR& evalTime) override
	{
		T result;
		interp_.interpolate(time_, data_, evalTime, result);
		return result;
	}

	//! returns the size of the trajectory
	/*!
	 * @return size of trajectory
	 */
	size_t size() const {
		if(data_.size() != time_.size())
			throw std::runtime_error("DiscreteTrajectoryBase: size inconsistent.");

		return this->data_.size();
	}

	//! get the first element
	T& front() {return data_.front();}

	//! get the first element
	const T& front() const {return data_.front();}

	//! get the last element
	T& back() {return data_.back();}

	//! get the last element
	const T& back() const {return data_.back();}

	//! access a certain index of the trajectory (does not interpolate)
	T& operator [](const size_t i) {return data_[i];}

	//! access a certain index of the trajectory (does not interpolate)
	const T& operator [](const size_t i) const {return data_[i];}


	//! get the time stamp of the first element
	const Time startTime() const {return time_.front();}

	//! get the time stamp of the last element
	const Time finalTime() const {return time_.back();}

	//! time duration of the trajectory
	const Time duration() const {return time_.back() - time_.front();}


	//! Add a data and time point at the end
	/*!
	 * Add a data-point and a time-point to the end of the trajectory.
	 * @param data
	 * @param time
	 * @param timeIsAbsolute
	 * 		true: time value is absolute, a simple push_back is sufficient
	 * 		false: time value is relative to current end of time trajectory
	 */
	void push_back(const T& data, const Time& time, const bool timeIsAbsolute){

		if(timeIsAbsolute)
			time_.push_back(time);
		else
			time_.push_back(time+time_.back());

		data_.push_back(data);
	}

	//! Remove the last data and time pair
	void pop_back(){
		time_.pop_back();
		data_.pop_back();
	}

	//! Erase front elements and optionally shift the trajectory in time
	/*!
	 * @param N		number of elements to erase from the front
	 * @param dt	(optional) shift the time trajectory about dt.
	 */
	void eraseFront(const size_t& N, const core::Time& dt = 0.0){
		time_.eraseFront(N);
		data_.eraseFront(N);
		shiftTime(dt);
	}

	//! Clear the trajectory
	void clear() {
		data_.clear();
		time_.clear();
	}

	//! Swap two trajectories
	/*!
	 * \warning swapping does not exchange the interpolation types, it just affects the data
	 *
	 * @param other trajectory to swap with
	 */
	void swapData(DiscreteTrajectoryBase& other){
		time_.swap(other.time_);
		data_.swap(other.data_);
	}

	//! assignment operator
	DiscreteTrajectoryBase& operator=(const DiscreteTrajectoryBase & other)
	{
	    if(this == &other)
	       return *this;

	    time_ = other.time_;
	    data_ = other.data_;
	    interp_.changeInterpolationType (other.interp_.getInterpolationType());

	    return *this;
	}

	//! get the time stamp at a certain index
	const SCALAR& getTimeFromIndex(const size_t& ind) const {return time_[ind];}

	//! get the index associated with a certain time
	/*!
	 * If the exact time is not stored, the interpolation will find the corresponding index.
	 * @param t time to search for
	 * @return according index
	 */
	size_t getIndexFromTime(const SCALAR& t) {return interp_.findIndex(time_, t);}

	//! get the data array
	DiscreteArray<T, Alloc>& getDataArray() {return data_;}

	//! get the data array
	const DiscreteArray<T, Alloc>& getDataArray() const {return data_;}

	//! get the time array
	tpl::TimeArray<SCALAR>& getTimeArray() {return time_;}

	//! get the time array
	const tpl::TimeArray<SCALAR>& getTimeArray() const {return time_;}

protected:

	tpl::TimeArray<SCALAR> time_; //!< time array

	DiscreteArray<T, Alloc> data_; //!< data array

	Interpolation<T, Alloc, SCALAR> interp_; //!< interpolation strategy

};

} /* namespace core */
} /* namespace ct */

#endif
