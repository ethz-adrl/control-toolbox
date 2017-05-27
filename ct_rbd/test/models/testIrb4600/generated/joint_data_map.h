#ifndef IIT_TESTIRB4600_JOINT_DATA_MAP_H_
#define IIT_TESTIRB4600_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace testirb4600 {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[JA] = rhs[JA];
    data[JB] = rhs[JB];
    data[JC] = rhs[JC];
    data[JD] = rhs[JD];
    data[JE] = rhs[JE];
    data[JF] = rhs[JF];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[JA] = value;
    data[JB] = value;
    data[JC] = value;
    data[JD] = value;
    data[JE] = value;
    data[JF] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   jA = "
    << map[JA]
    << "   jB = "
    << map[JB]
    << "   jC = "
    << map[JC]
    << "   jD = "
    << map[JD]
    << "   jE = "
    << map[JE]
    << "   jF = "
    << map[JF]
    ;
    return out;
}

}
}
#endif
