#ifndef IIT_ROBOT_CT_QUADROTOR_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_CT_QUADROTOR_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace ct_quadrotor {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot ct_quadrotor.
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
        const IMatrix& getTensor_body() const;
        const IMatrix& getTensor_link1() const;
        const IMatrix& getTensor_link2() const;
        SCALAR getMass_body() const;
        SCALAR getMass_link1() const;
        SCALAR getMass_link2() const;
        const Vec3d& getCOM_body() const;
        const Vec3d& getCOM_link1() const;
        const Vec3d& getCOM_link2() const;
        SCALAR getTotalMass() const;

    private:

        IMatrix tensor_body;
        IMatrix tensor_link1;
        IMatrix tensor_link2;
        Vec3d com_body;
        Vec3d com_link1;
        Vec3d com_link2;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_body() const {
    return this->tensor_body;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link1() const {
    return this->tensor_link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link2() const {
    return this->tensor_link2;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getMass_body() const {
    return this->tensor_body.getMass();
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
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_body() const {
    return this->com_body;
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
inline typename InertiaProperties<TRAIT>::SCALAR InertiaProperties<TRAIT>::getTotalMass() const {
    return 0.5 + 0.025 + 0.1;
}

} // namespace tpl

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
