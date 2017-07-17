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

#ifndef CT_DISCRETE_ARRAY_H_
#define CT_DISCRETE_ARRAY_H_

#include <vector>
#include <iostream>

namespace ct {
namespace core {

//! An discrete array (vector) of a particular data type.
/*!
 * This class is a discrete array/vector/sequence of data of a particular type.
 *
 * \tparam T data type
 * \tparam Alloc memory allocator type
 */
template <class T, class Alloc = Eigen::aligned_allocator<T>>
class DiscreteArray : private std::vector<T, Alloc> { // private since we only want to expose selective functionality
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef T value_type; //!< data type
	typedef std::vector<T, Alloc > Base; //!< base class (std::vector)
	typedef typename std::vector<T, Alloc >::iterator iterator; //!< iterator
	typedef typename std::vector<T, Alloc >::const_iterator const_iterator; //!< const iterator

	//! default constructor
	DiscreteArray() :
		Base(Alloc())
	{};

	//! move constructor
	DiscreteArray(const DiscreteArray&& other) :
			Base(static_cast<const Base&&>(other))
	{
	}

	//! copy constructor
	DiscreteArray(const DiscreteArray& other) :
		Base(static_cast<const Base&>(other))
	{};

	//! resize constructor
	/*!
	 * initializes an array with a certain length and fills it with a default value
	 * @param n length of array
	 * @param value default value
	 */
	DiscreteArray(int n, const T& value=T())
		: Base(n, value)  {};

	//! copy constructor
	/*!
	 * Copies a segment of another array
	 * @param first iterator to first element
	 * @param last iterator to last element
	 */
	DiscreteArray(iterator first, iterator last)
		: Base(first, last) {};

	//! copy constructor
	/*!
	 * Copies a segment of another array
	 * @param first iterator to first element
	 * @param last iterator to last element
	 */
	DiscreteArray(const_iterator first, const_iterator last)
		: Base(first, last) {};

	//! destructor
	virtual ~DiscreteArray() {};

	using Base::operator[];
	using Base::size;
	using Base::resize;
	using Base::reserve;
	using Base::at;
	using Base::pop_back;
	using Base::push_back;
	using Base::clear;
	using Base::front;
	using Base::back;
	using Base::begin;
	using Base::end;

	//! swaps the content of two arrays
	/*!
	 * @param other reference to the other array
	 */
	void swap(DiscreteArray& other)
	{
		Base::swap(static_cast<Base&>(other));
	}

	//!< assignment operator
	DiscreteArray<T, Alloc>& operator=(const DiscreteArray<T, Alloc> &rhs) {
	    // Check for self-assignment!
	    if (this == &rhs)
	      return *this;

	    Base::operator=(static_cast<const Base&>(rhs));

	    return *this;
	}

	//! returns the underlying std::vector
	Base& toImplementation() {return *this;}

	//! returns the underlying std::vector
	const Base& toImplementation() const {return *this;}

	//! rerase an element from the front
	void eraseFront(const size_t N) {this->erase(this->begin(), this->begin()+N);}

	//! sets all elements to zero.
	virtual void setZero() {std::fill(this->begin(), this->end(), T::Zero());}

	//! addas an offest to each element
	void addOffset(const T& offset) {std::for_each(this->begin(), this->end(), [&](T& val) { val+=offset;});}

private:

};

} /* namespace core */
} /* namespace ct */

#endif /* DISCRETETRAJECTORYBASE_H_ */
