#ifndef IIT_CT_DOUBLEINVERTEDPENDULUM_LINK_DATA_MAP_H_
#define IIT_CT_DOUBLEINVERTEDPENDULUM_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace ct_DoubleInvertedPendulum {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[DOUBLEINVERTEDPENDULUMBASE] = rhs[DOUBLEINVERTEDPENDULUMBASE];
    data[LINK1] = rhs[LINK1];
    data[LINK2] = rhs[LINK2];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[DOUBLEINVERTEDPENDULUMBASE] = value;
    data[LINK1] = value;
    data[LINK2] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   DoubleInvertedPendulumBase = "
    << map[DOUBLEINVERTEDPENDULUMBASE]
    << "   Link1 = "
    << map[LINK1]
    << "   Link2 = "
    << map[LINK2]
    ;
    return out;
}

}
}
#endif
