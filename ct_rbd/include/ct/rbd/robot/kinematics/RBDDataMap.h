/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

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
