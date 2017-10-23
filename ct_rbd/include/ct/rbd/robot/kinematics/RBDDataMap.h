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

/**
 * @file RBDDataMap.h
 * @author taken from iit rbd dynamics
 */

#ifndef _RBDDATAMAP_H_
#define _RBDDATAMAP_H_

#include <cstring>

namespace ct {
namespace rbd {

/**
 * \brief A very simple container to associate N generic data item T
 */
template <typename T, size_t N>
class RBDDataMap
{
private:
	T data[N];

public:
	RBDDataMap(){};
	RBDDataMap(const T& defaultValue);
	RBDDataMap(const RBDDataMap& rhs);
	RBDDataMap& operator=(const RBDDataMap& rhs);
	RBDDataMap& operator=(const T& rhs);
	T& operator[](size_t which);
	const T& operator[](size_t which) const;

private:
	void copydata(const RBDDataMap& rhs);
	void assigndata(const T& commonValue);
};

template <typename T, size_t N>
inline RBDDataMap<T, N>::RBDDataMap(const T& value)
{
	assigndata(value);
}

template <typename T, size_t N>
inline RBDDataMap<T, N>::RBDDataMap(const RBDDataMap& rhs)
{
	copydata(rhs);
}

template <typename T, size_t N>
inline RBDDataMap<T, N>& RBDDataMap<T, N>::operator=(const RBDDataMap& rhs)
{
	if (&rhs != this)
	{
		copydata(rhs);
	}
	return *this;
}

template <typename T, size_t N>
inline RBDDataMap<T, N>& RBDDataMap<T, N>::operator=(const T& value)
{
	assigndata(value);
	return *this;
}

template <typename T, size_t N>
inline T& RBDDataMap<T, N>::operator[](size_t l)
{
	return data[l];
}

template <typename T, size_t N>
inline const T& RBDDataMap<T, N>::operator[](size_t l) const
{
	return data[l];
}

template <typename T, size_t N>
inline void RBDDataMap<T, N>::copydata(const RBDDataMap& rhs)
{
	for (size_t id = 0; id < N; ++id)
		data[id] = rhs[id];
}

template <typename T, size_t N>
inline void RBDDataMap<T, N>::assigndata(const T& value)
{
	for (size_t id = 0; id < N; ++id)
		data[id] = value;
}

template <typename T, size_t N>
inline std::ostream& operator<<(std::ostream& out, const RBDDataMap<T, N>& map)
{
	for (size_t id = 0; id < N; ++id)
	{
		out << "[" << id << "] = " << map[id] << " ";
	}
	return out;
}

}  // namespace rbd
}  // namespace ct

#endif /* _RBDDATAMAP_H_ */
