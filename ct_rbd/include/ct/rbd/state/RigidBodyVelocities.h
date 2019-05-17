/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \brief Representation of Rigid Body Velocities, currently just a typedef
 * \ingroup State
 */
// unfortunately kindr stores linear velocity first and not rotational velocity.
// so we cannot use it directly. Instead we privately inherit and make the "safe"
// methods public and "override" the others.
template <typename SCALAR = double>
class RigidBodyVelocities : private kindr::TwistLinearVelocityLocalAngularVelocity<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, 6, 1> Vector6;

    using kindr::TwistLinearVelocityLocalAngularVelocity<SCALAR>::getTranslationalVelocity;
    using kindr::TwistLinearVelocityLocalAngularVelocity<SCALAR>::getRotationalVelocity;
    using kindr::TwistLinearVelocityLocalAngularVelocity<SCALAR>::setZero;

    Vector6 getVector() const
    {
        Vector6 vector6;
        vector6 << getRotationalVelocity().toImplementation(), getTranslationalVelocity().toImplementation();
        return vector6;
    }

    void setVector(const Vector6& vector6)
    {
        getRotationalVelocity().toImplementation() = vector6.template head<3>();
        getTranslationalVelocity().toImplementation() = vector6.template tail<3>();
    }

private:
};

}  // namespace tpl

// convenience typedef (required)
typedef tpl::RigidBodyVelocities<double> RigidBodyVelocities;

} /* namespace rbd */
} /* namespace ct */
