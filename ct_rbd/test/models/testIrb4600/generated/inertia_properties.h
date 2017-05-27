#ifndef IIT_ROBOT_TESTIRB4600_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_TESTIRB4600_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace testirb4600 {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot testirb4600.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl{

template<typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar SCALAR;

        typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> IMatrix;
        typedef Eigen::Matrix<SCALAR, 3, 1> Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_link1() const;
        const IMatrix& getTensor_link2() const;
        const IMatrix& getTensor_link3() const;
        const IMatrix& getTensor_link4() const;
        const IMatrix& getTensor_link5() const;
        const IMatrix& getTensor_link6() const;
        SCALAR getMass_link1() const;
        SCALAR getMass_link2() const;
        SCALAR getMass_link3() const;
        SCALAR getMass_link4() const;
        SCALAR getMass_link5() const;
        SCALAR getMass_link6() const;
        const Vec3d& getCOM_link1() const;
        const Vec3d& getCOM_link2() const;
        const Vec3d& getCOM_link3() const;
        const Vec3d& getCOM_link4() const;
        const Vec3d& getCOM_link5() const;
        const Vec3d& getCOM_link6() const;
        SCALAR getTotalMass() const;

    private:

        IMatrix tensor_link1;
        IMatrix tensor_link2;
        IMatrix tensor_link3;
        IMatrix tensor_link4;
        IMatrix tensor_link5;
        IMatrix tensor_link6;
        Vec3d com_link1;
        Vec3d com_link2;
        Vec3d com_link3;
        Vec3d com_link4;
        Vec3d com_link5;
        Vec3d com_link6;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link1() const {
    return this->tensor_link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link2() const {
    return this->tensor_link2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link3() const {
    return this->tensor_link3;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link4() const {
    return this->tensor_link4;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link5() const {
    return this->tensor_link5;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link6() const {
    return this->tensor_link6;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_link1() const {
    return this->tensor_link1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_link2() const {
    return this->tensor_link2.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_link3() const {
    return this->tensor_link3.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_link4() const {
    return this->tensor_link4.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_link5() const {
    return this->tensor_link5.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_link6() const {
    return this->tensor_link6.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link1() const {
    return this->com_link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link2() const {
    return this->com_link2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link3() const {
    return this->com_link3;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link4() const {
    return this->com_link4;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link5() const {
    return this->com_link5;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link6() const {
    return this->com_link6;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getTotalMass() const {
    return 120.0 + 120.0 + 120.0 + 40.0 + 10.0 + 5.0;
}

} // namespace tpl

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
