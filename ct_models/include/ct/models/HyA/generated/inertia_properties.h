#ifndef IIT_ROBOT_CT_HYA_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_CT_HYA_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace ct_HyA {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot ct_HyA.
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
        const IMatrix& getTensor_Shoulder_AA() const;
        const IMatrix& getTensor_Shoulder_FE() const;
        const IMatrix& getTensor_Humerus_R() const;
        const IMatrix& getTensor_Elbow_FE() const;
        const IMatrix& getTensor_Wrist_R() const;
        const IMatrix& getTensor_Wrist_FE() const;
        SCALAR getMass_Shoulder_AA() const;
        SCALAR getMass_Shoulder_FE() const;
        SCALAR getMass_Humerus_R() const;
        SCALAR getMass_Elbow_FE() const;
        SCALAR getMass_Wrist_R() const;
        SCALAR getMass_Wrist_FE() const;
        const Vec3d& getCOM_Shoulder_AA() const;
        const Vec3d& getCOM_Shoulder_FE() const;
        const Vec3d& getCOM_Humerus_R() const;
        const Vec3d& getCOM_Elbow_FE() const;
        const Vec3d& getCOM_Wrist_R() const;
        const Vec3d& getCOM_Wrist_FE() const;
        SCALAR getTotalMass() const;

    private:

        IMatrix tensor_Shoulder_AA;
        IMatrix tensor_Shoulder_FE;
        IMatrix tensor_Humerus_R;
        IMatrix tensor_Elbow_FE;
        IMatrix tensor_Wrist_R;
        IMatrix tensor_Wrist_FE;
        Vec3d com_Shoulder_AA;
        Vec3d com_Shoulder_FE;
        Vec3d com_Humerus_R;
        Vec3d com_Elbow_FE;
        Vec3d com_Wrist_R;
        Vec3d com_Wrist_FE;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Shoulder_AA() const {
    return this->tensor_Shoulder_AA;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Shoulder_FE() const {
    return this->tensor_Shoulder_FE;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Humerus_R() const {
    return this->tensor_Humerus_R;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Elbow_FE() const {
    return this->tensor_Elbow_FE;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Wrist_R() const {
    return this->tensor_Wrist_R;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_Wrist_FE() const {
    return this->tensor_Wrist_FE;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_Shoulder_AA() const {
    return this->tensor_Shoulder_AA.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_Shoulder_FE() const {
    return this->tensor_Shoulder_FE.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_Humerus_R() const {
    return this->tensor_Humerus_R.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_Elbow_FE() const {
    return this->tensor_Elbow_FE.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_Wrist_R() const {
    return this->tensor_Wrist_R.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_Wrist_FE() const {
    return this->tensor_Wrist_FE.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Shoulder_AA() const {
    return this->com_Shoulder_AA;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Shoulder_FE() const {
    return this->com_Shoulder_FE;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Humerus_R() const {
    return this->com_Humerus_R;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Elbow_FE() const {
    return this->com_Elbow_FE;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Wrist_R() const {
    return this->com_Wrist_R;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_Wrist_FE() const {
    return this->com_Wrist_FE;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getTotalMass() const {
    return 2.688 + 2.5924191 + 2.327 + 1.7423722 + 2.1032 + 1.547475;
}

} // namespace tpl

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
