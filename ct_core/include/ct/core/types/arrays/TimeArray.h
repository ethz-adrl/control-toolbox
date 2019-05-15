/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../Time.h"
#include <ct/core/common/linspace.h>

#include "ScalarArray.h"

namespace ct {
namespace core {
namespace tpl {

//! An array in time
/*!
 * \todo This is just a duplicate of a scalar array. Remove.
 */
template <typename SCALAR>
class TimeArray : public ScalarArray<SCALAR>
{
public:
    //! default constructor
    TimeArray() {}
    //! evenly spaced constructor
    /*!
	 * \param dt	the time spacing
	 * \param N		the number of points, minimum 2
	 * \param t0	the starting time, defaults to zero
	 */
    TimeArray(const SCALAR& dt, const size_t& N, const SCALAR& t0 = 0.0)
        : ScalarArray<SCALAR>(linspace<ScalarArray<SCALAR>>(t0, t0 + (N - 1) * dt, N))
    {
    }

    //! resize constructor
    /*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
    TimeArray(size_t n, const SCALAR& value = SCALAR()) : ScalarArray<SCALAR>(n, value){};

    //! copy constructor
    TimeArray(const TimeArray& other) : ScalarArray<SCALAR>(other){};

    //! std::vector constructor
    TimeArray(const std::vector<SCALAR>& arg) : ScalarArray<SCALAR>()
    {
        for (size_t i = 0; i < arg.size(); i++)
            this->push_back(arg[i]);
    }
};
}

typedef tpl::TimeArray<double> TimeArray;
}
}
