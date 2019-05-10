/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/arrays/DiscreteArray.h>
#include <ct/core/types/arrays/TimeArray.h>

namespace ct {
namespace core {

enum InterpolationType
{
    ZOH = 0,  //!< Zero-Order hold
    LIN       //!< LIN
};

//! Class that performs interpolation of data in time
/*!
 * This class performs interpolation of a data array based on timestamps
 *
 * @tparam Data_T the data type for interpolation, e.g. a scalar or a StateVector
 * @tparam Alloc_Data An optional allocator for the data type
 * @tparam Alloc_Time An optional allocator for the time data type
 */
template <typename Data_T, class Alloc_Data = Eigen::aligned_allocator<Data_T>, typename SCALAR = double>
class Interpolation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DiscreteArray<Data_T, Alloc_Data> DiscreteArray_t;

    //! Default constructor
    /*!
	 * \warning This does not initialize the underlying data or time series and thus all
	 * evaluating calls will fail.
	 *
	 * @param type The interpolation strategy to use
	 */
    Interpolation(const InterpolationType& type = LIN) : index_(0), type_(type) {}
    //! Copy Constructor
    Interpolation(const Interpolation& arg) : index_(arg.index_), type_(arg.type_) {}
    //! This method performs the interpolation
    /*!
	 * @param timeArray timing information of the data points
	 * @param dataArray	the data points in form of a DiscreteArray
	 * @param enquiryTime the time where to evaluate the interpolation
	 * @param enquiryData the result of the interpolation
	 * @param greatestLessTimeStampIndex the smallest index corresponding to a time smaller than the inquired Time
	 */
    void interpolate(const tpl::TimeArray<SCALAR>& timeArray,
        const DiscreteArray_t& dataArray,
        const SCALAR& enquiryTime,
        Data_T& enquiryData,
        int greatestLessTimeStampIndex = -1)
    {
        if (timeArray.size() == 0)
            throw std::runtime_error("Interpolation.h : TimeArray is size 0.");

        if (dataArray.size() == 0)
            throw std::runtime_error("Interpolation.h : DataArray is size 0.");

        if (timeArray.size() != dataArray.size())
            throw std::runtime_error("Interpolation.h : The size of timeStamp vector (=" +
                                     std::to_string(timeArray.size()) + ") is not equal to the size of data vector (=" +
                                     std::to_string(dataArray.size()) + ").");


        // treat special case of trajectory length equal 1
        if (dataArray.size() == 1)
        {
            enquiryData = dataArray.front();
            return;
        }

        int ind;
        if (greatestLessTimeStampIndex == -1)
            ind = findIndex(timeArray, enquiryTime);
        else
        {
            ind = greatestLessTimeStampIndex;
            index_ = greatestLessTimeStampIndex;
        }

        if (enquiryTime < timeArray.front())
        {
            enquiryData = dataArray.front();
            return;
        }

        if (ind == (int)timeArray.size() - 1)
        {
            enquiryData = dataArray.back();
            return;
        }

        SCALAR alpha = (enquiryTime - timeArray.at(ind + 1)) / (timeArray.at(ind) - timeArray.at(ind + 1));


        if (type_ == InterpolationType::LIN)
            enquiryData = alpha * dataArray.at(ind) + (1 - alpha) * dataArray.at(ind + 1);
        else if (type_ == InterpolationType::ZOH)
            enquiryData = dataArray.at(ind);
        else
            throw std::runtime_error("Unknown Interpolation type!");
    }


    //! access the greatest index which is smaller than the inquired interpolation time
    int getGreatestLessTimeStampIndex() { return index_; }
    //! get the employed interpolation type
    InterpolationType getInterpolationType() const { return type_; }
    //! change the interpolation type
    void changeInterpolationType(const InterpolationType& type) { type_ = type; }
    //! find an index corresponding to a certain inquiry time
    int findIndex(const tpl::TimeArray<SCALAR>& timeArray, const SCALAR& enquiryTime)
    {
        int index = -1;

        index_ = std::min(index_, (int)timeArray.size() - 1);

        if (timeArray.at(index_) > enquiryTime)
        {
            for (int i = index_; i >= 0; i--)
            {
                index = i;
                if (timeArray.at(i) <= enquiryTime)
                    break;
            }
        }
        else
        {
            for (int i = index_; i < (int)timeArray.size(); i++)
            {
                index = i;
                if (timeArray.at(i) > enquiryTime)
                {
                    index = i - 1;
                    break;
                }
            }
        }

        // throw error if index is wrong
        if (index < 0)
            throw std::runtime_error(
                "Interpolation.h : index in protected member findIndex((const SCALAR& enquiryTime) not computed "
                "properly");

        index_ = index;

        return index;
    }


protected:
    int index_;

    InterpolationType type_;
};


}  // namespace ct
}  // namespace core
