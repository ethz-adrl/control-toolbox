#ifndef IIT_CT_HYA_LINK_DATA_MAP_H_
#define IIT_CT_HYA_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace ct_HyA {

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
    data[HYABASE] = rhs[HYABASE];
    data[SHOULDER_AA] = rhs[SHOULDER_AA];
    data[SHOULDER_FE] = rhs[SHOULDER_FE];
    data[HUMERUS_R] = rhs[HUMERUS_R];
    data[ELBOW_FE] = rhs[ELBOW_FE];
    data[WRIST_R] = rhs[WRIST_R];
    data[WRIST_FE] = rhs[WRIST_FE];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[HYABASE] = value;
    data[SHOULDER_AA] = value;
    data[SHOULDER_FE] = value;
    data[HUMERUS_R] = value;
    data[ELBOW_FE] = value;
    data[WRIST_R] = value;
    data[WRIST_FE] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   HyABase = "
    << map[HYABASE]
    << "   Shoulder_AA = "
    << map[SHOULDER_AA]
    << "   Shoulder_FE = "
    << map[SHOULDER_FE]
    << "   Humerus_R = "
    << map[HUMERUS_R]
    << "   Elbow_FE = "
    << map[ELBOW_FE]
    << "   Wrist_R = "
    << map[WRIST_R]
    << "   Wrist_FE = "
    << map[WRIST_FE]
    ;
    return out;
}

}
}
#endif
