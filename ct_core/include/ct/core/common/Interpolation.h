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

#ifndef CT_CORE_INTERPOLATION_H_
#define CT_CORE_INTERPOLATION_H_

#include <ct/core/types/trajectories/TimeArray.h>
#include <ct/core/types/trajectories/DiscreteArray.h>

namespace ct{
namespace core{

enum InterpolationType {
	ZOH = 0,//!< Zero-Order hold
	LIN     //!< LIN
};

//! Class that performs interpolation of data in time
/*!
 * This class performs interpolation of a data array based on timestamps
 *
 * @tparam Data_T the data type for interpolation, e.g. a scalar or a StateVector
 * @tparam Alloc_Data An optional allocator for the data type
 * @tparam Alloc_Time An optional allocator for the time data type
 */
template <typename Data_T, class Alloc_Data=Eigen::aligned_allocator<Data_T>, class Alloc_Time=Eigen::aligned_allocator<double>>
class Interpolation {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteArray<Data_T,Alloc_Data> DiscreteArray_t;

	//! Default constructor
	/*!
	 * \warning This does not initialize the underlying data or time series and thus all
	 * evaluating calls will fail.
	 *
	 * @param type The interpolation strategy to use
	 */
	Interpolation(const InterpolationType& type = LIN)

	: index_(0),
	  zeroFunction_(false),
	  timeStampPtr_(NULL),
	  dataPtr_(NULL),
	  type_(type)
	{}

	//! Constructor with time and data array
	/*!
	 * Initializes the interpolator with a time and data array (vector)
	 * @param timeStampPtr The time array
	 * @param dataPtr The data array
	 * @param type The interpolation strategy
	 */
	Interpolation(const TimeArray* timeStampPtr, const DiscreteArray_t* dataPtr, const InterpolationType& type = LIN)

	: index_(0),
	  zeroFunction_(false),
	  timeStampPtr_(timeStampPtr),
	  dataPtr_(dataPtr),
	  type_(type)
	{
		// todo: check if time and data are the same length
	}

	Interpolation(const Interpolation& arg):
		index_(arg.index_),
		zeroFunction_(arg.zeroFunction_),
		timeStampPtr_(arg.timeStampPtr_),
		dataPtr_(arg.dataPtr_),
		type_(arg.type_)
	{}

	void reset()
	{
		index_ = 0;
		zeroFunction_ = false;
	}

	void setTimeStamp(const TimeArray* timeStampPtr)
	{
		reset();
		timeStampPtr_ = timeStampPtr;
	}

	void setData(const DiscreteArray_t* dataPtr)
	{
		reset();
		dataPtr_ = dataPtr;
	}

	void setZero()
	{
		reset();
		zeroFunction_ = true;
	}


	const TimeArray* getTimeStampPtr() {return timeStampPtr_;}


	const DiscreteArray_t* getDataPtr() {return dataPtr_;}


	void interpolate(const Time& enquiryTime, Data_T& enquiryData, int greatestLessTimeStampIndex = -1) {

		if (zeroFunction_==true)  {
			enquiryData.setZero();
			return;
		}

		if (timeStampPtr_==NULL)  throw std::runtime_error("timeStampPtr is not initialized.");
		if (dataPtr_==NULL)       throw std::runtime_error("dataPtr is not initialized.");

		if (timeStampPtr_->size()==0)
			throw std::runtime_error("Interpolation.h : Interpolation is not initialized.");

		if (timeStampPtr_->size()!=dataPtr_->size())
			throw std::runtime_error("Interpolation.h : The size of timeStamp vector (="+std::to_string(timeStampPtr_->size())+") is not equal to the size of data vector (="+std::to_string(dataPtr_->size())+").");

		if (timeStampPtr_->size()==1)  {
			enquiryData = dataPtr_->front();
			return;
		}

		size_t ind;
		if (greatestLessTimeStampIndex == -1)
			ind = findIndex(enquiryTime);
		else {
			ind = greatestLessTimeStampIndex;
			index_ = greatestLessTimeStampIndex;
		}

		if (enquiryTime<timeStampPtr_->front()) {
			enquiryData = dataPtr_->front();
			return;
		}

		if (ind==timeStampPtr_->size()-1) {
			enquiryData = dataPtr_->back();
			return;
		}

		double alpha = (enquiryTime-timeStampPtr_->at(ind+1)) / (timeStampPtr_->at(ind)-timeStampPtr_->at(ind+1));


		if(type_ == InterpolationType::LIN)
			enquiryData = alpha*dataPtr_->at(ind) + (1-alpha)*dataPtr_->at(ind+1);
		else if(type_ == InterpolationType::ZOH)
			enquiryData = dataPtr_->at(ind);
		else
			throw std::runtime_error("Unknown Interpolation type!");
	}


	size_t getGreatestLessTimeStampIndex() { return index_; }

	InterpolationType getInterpolationType() const {return type_;}

	void changeInterpolationType(const InterpolationType& type){ type_ = type;}


	size_t findIndex(const double& enquiryTime) {

		int index = -1;

		if (timeStampPtr_->at(index_) > enquiryTime) {
			for (int i=index_; i>=0; i--)  {
				index = i;
				if (timeStampPtr_->at(i) <= enquiryTime)
					break;
			}
		} else {
			for (int i=index_; i<timeStampPtr_->size(); i++) {
				index = i;
				if (timeStampPtr_->at(i) > enquiryTime) {
					index = i-1;
					break;
				}
			}
		}

		// throw error if index is wrong
		if(index < 0)
			throw std::runtime_error("Interpolation.h : index in protected member findIndex((const double& enquiryTime) not computed properly");

		index_ = index;

		return index;
	}


protected:

	size_t index_;
	bool zeroFunction_;

	const TimeArray* timeStampPtr_;
	const DiscreteArray_t* dataPtr_;

	InterpolationType type_;

};


} // namespace ct
} // namespace core

#endif /* Interpolation_H_ */
