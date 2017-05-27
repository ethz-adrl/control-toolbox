#ifndef IIT_TESTIRB4600_LINK_DATA_MAP_H_
#define IIT_TESTIRB4600_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace testirb4600 {

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
    data[LINK0] = rhs[LINK0];
    data[LINK1] = rhs[LINK1];
    data[LINK2] = rhs[LINK2];
    data[LINK3] = rhs[LINK3];
    data[LINK4] = rhs[LINK4];
    data[LINK5] = rhs[LINK5];
    data[LINK6] = rhs[LINK6];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[LINK0] = value;
    data[LINK1] = value;
    data[LINK2] = value;
    data[LINK3] = value;
    data[LINK4] = value;
    data[LINK5] = value;
    data[LINK6] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   link0 = "
    << map[LINK0]
    << "   link1 = "
    << map[LINK1]
    << "   link2 = "
    << map[LINK2]
    << "   link3 = "
    << map[LINK3]
    << "   link4 = "
    << map[LINK4]
    << "   link5 = "
    << map[LINK5]
    << "   link6 = "
    << map[LINK6]
    ;
    return out;
}

}
}
#endif
