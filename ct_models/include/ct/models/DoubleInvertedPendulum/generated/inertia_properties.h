#ifndef IIT_ROBOT_CT_DOUBLEINVERTEDPENDULUM_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_CT_DOUBLEINVERTEDPENDULUM_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace ct_DoubleInvertedPendulum {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot ct_DoubleInvertedPendulum.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_Link1() const;
        const IMatrix& getTensor_Link2() const;
        Scalar getMass_Link1() const;
        Scalar getMass_Link2() const;
        const Vec3d& getCOM_Link1() const;
        const Vec3d& getCOM_Link2() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_Link1;
        IMatrix tensor_Link2;
        Vec3d com_Link1;
        Vec3d com_Link2;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Link1() const {
    return this->tensor_Link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Link2() const {
    return this->tensor_Link2;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_Link1() const {
    return this->tensor_Link1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_Link2() const {
    return this->tensor_Link2.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Link1() const {
    return this->com_Link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Link2() const {
    return this->com_Link2;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 1.0 + 1.0;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
