#ifndef IIT_ROBOT_HYQ_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_HYQ_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace HyQ {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot HyQ.
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
        const IMatrix& getTensor_trunk() const;
        const IMatrix& getTensor_LF_hipassembly() const;
        const IMatrix& getTensor_LF_upperleg() const;
        const IMatrix& getTensor_LF_lowerleg() const;
        const IMatrix& getTensor_RF_hipassembly() const;
        const IMatrix& getTensor_RF_upperleg() const;
        const IMatrix& getTensor_RF_lowerleg() const;
        const IMatrix& getTensor_LH_hipassembly() const;
        const IMatrix& getTensor_LH_upperleg() const;
        const IMatrix& getTensor_LH_lowerleg() const;
        const IMatrix& getTensor_RH_hipassembly() const;
        const IMatrix& getTensor_RH_upperleg() const;
        const IMatrix& getTensor_RH_lowerleg() const;
        Scalar getMass_trunk() const;
        Scalar getMass_LF_hipassembly() const;
        Scalar getMass_LF_upperleg() const;
        Scalar getMass_LF_lowerleg() const;
        Scalar getMass_RF_hipassembly() const;
        Scalar getMass_RF_upperleg() const;
        Scalar getMass_RF_lowerleg() const;
        Scalar getMass_LH_hipassembly() const;
        Scalar getMass_LH_upperleg() const;
        Scalar getMass_LH_lowerleg() const;
        Scalar getMass_RH_hipassembly() const;
        Scalar getMass_RH_upperleg() const;
        Scalar getMass_RH_lowerleg() const;
        const Vec3d& getCOM_trunk() const;
        const Vec3d& getCOM_LF_hipassembly() const;
        const Vec3d& getCOM_LF_upperleg() const;
        const Vec3d& getCOM_LF_lowerleg() const;
        const Vec3d& getCOM_RF_hipassembly() const;
        const Vec3d& getCOM_RF_upperleg() const;
        const Vec3d& getCOM_RF_lowerleg() const;
        const Vec3d& getCOM_LH_hipassembly() const;
        const Vec3d& getCOM_LH_upperleg() const;
        const Vec3d& getCOM_LH_lowerleg() const;
        const Vec3d& getCOM_RH_hipassembly() const;
        const Vec3d& getCOM_RH_upperleg() const;
        const Vec3d& getCOM_RH_lowerleg() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_trunk;
        IMatrix tensor_LF_hipassembly;
        IMatrix tensor_LF_upperleg;
        IMatrix tensor_LF_lowerleg;
        IMatrix tensor_RF_hipassembly;
        IMatrix tensor_RF_upperleg;
        IMatrix tensor_RF_lowerleg;
        IMatrix tensor_LH_hipassembly;
        IMatrix tensor_LH_upperleg;
        IMatrix tensor_LH_lowerleg;
        IMatrix tensor_RH_hipassembly;
        IMatrix tensor_RH_upperleg;
        IMatrix tensor_RH_lowerleg;
        Vec3d com_trunk;
        Vec3d com_LF_hipassembly;
        Vec3d com_LF_upperleg;
        Vec3d com_LF_lowerleg;
        Vec3d com_RF_hipassembly;
        Vec3d com_RF_upperleg;
        Vec3d com_RF_lowerleg;
        Vec3d com_LH_hipassembly;
        Vec3d com_LH_upperleg;
        Vec3d com_LH_lowerleg;
        Vec3d com_RH_hipassembly;
        Vec3d com_RH_upperleg;
        Vec3d com_RH_lowerleg;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_trunk() const {
    return this->tensor_trunk;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_hipassembly() const {
    return this->tensor_LF_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_upperleg() const {
    return this->tensor_LF_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_lowerleg() const {
    return this->tensor_LF_lowerleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_hipassembly() const {
    return this->tensor_RF_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_upperleg() const {
    return this->tensor_RF_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_lowerleg() const {
    return this->tensor_RF_lowerleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_hipassembly() const {
    return this->tensor_LH_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_upperleg() const {
    return this->tensor_LH_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_lowerleg() const {
    return this->tensor_LH_lowerleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_hipassembly() const {
    return this->tensor_RH_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_upperleg() const {
    return this->tensor_RH_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_lowerleg() const {
    return this->tensor_RH_lowerleg;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_trunk() const {
    return this->tensor_trunk.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_hipassembly() const {
    return this->tensor_LF_hipassembly.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_upperleg() const {
    return this->tensor_LF_upperleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_lowerleg() const {
    return this->tensor_LF_lowerleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_hipassembly() const {
    return this->tensor_RF_hipassembly.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_upperleg() const {
    return this->tensor_RF_upperleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_lowerleg() const {
    return this->tensor_RF_lowerleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_hipassembly() const {
    return this->tensor_LH_hipassembly.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_upperleg() const {
    return this->tensor_LH_upperleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_lowerleg() const {
    return this->tensor_LH_lowerleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_hipassembly() const {
    return this->tensor_RH_hipassembly.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_upperleg() const {
    return this->tensor_RH_upperleg.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_lowerleg() const {
    return this->tensor_RH_lowerleg.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_trunk() const {
    return this->com_trunk;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_hipassembly() const {
    return this->com_LF_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_upperleg() const {
    return this->com_LF_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_lowerleg() const {
    return this->com_LF_lowerleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_hipassembly() const {
    return this->com_RF_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_upperleg() const {
    return this->com_RF_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_lowerleg() const {
    return this->com_RF_lowerleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_hipassembly() const {
    return this->com_LH_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_upperleg() const {
    return this->com_LH_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_lowerleg() const {
    return this->com_LH_lowerleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_hipassembly() const {
    return this->com_RH_hipassembly;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_upperleg() const {
    return this->com_RH_upperleg;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_lowerleg() const {
    return this->com_RH_lowerleg;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 53.433 + 3.44 + 3.146 + 0.881 + 3.44 + 3.146 + 0.881 + 3.44 + 3.146 + 0.881 + 3.44 + 3.146 + 0.881;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
