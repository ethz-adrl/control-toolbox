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

#ifndef CT_TIME_ARRAY_H_
#define CT_TIME_ARRAY_H_

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
	TimeArray(){}

	//! evenly spaced constructor
	/*!
	 * \param dt	the time spacing
	 * \param N		the number of points, minimum 2
	 * \param t0	the starting time, defaults to zero
	 */
	TimeArray(const SCALAR& dt, const size_t& N, const SCALAR& t0 = 0.0):
		ScalarArray<SCALAR>(linspace<ScalarArray<SCALAR>>(t0, t0+(N-1)*dt, N))
		{}

	//! resize constructor
	/*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
	TimeArray(size_t n, const SCALAR& value=SCALAR())
	: ScalarArray<SCALAR>(n,value)  {};

	//! copy constructor
	TimeArray(const TimeArray& other)
	: ScalarArray<SCALAR>(other) {};

	//! std::vector constructor
	TimeArray(const std::vector<SCALAR>& arg):
		ScalarArray<SCALAR>()
	{
		for(size_t i = 0; i<arg.size(); i++)
			this->push_back(arg[i]);
	}
};

}

typedef tpl::TimeArray<double> TimeArray;

}
}

#endif /* TIMETRAJECTORY_H_ */
