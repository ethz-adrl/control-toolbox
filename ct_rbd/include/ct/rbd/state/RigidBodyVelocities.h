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
template <typename SCALAR = double>
class RigidBodyVelocities
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3;
    typedef Eigen::Matrix<SCALAR, 6, 1> Vector6;

    typedef Eigen::VectorBlock<Vector6, 3> Vector3Block;
    typedef Eigen::VectorBlock<const Vector6, 3> Vector3BlockConst;

    void setZero() { twist_.setZero(); }

    Vector3Block getRotationalVelocity() { return twist_.template head<3>(); }
    const Vector3BlockConst getRotationalVelocity() const { return twist_.template head<3>(); }

    Vector3Block getTranslationalVelocity() { return twist_.template tail<3>(); }
    const Vector3BlockConst getTranslationalVelocity() const { return twist_.template tail<3>(); }

    Vector6& getVector() { return twist_; }
    const Vector6& getVector() const { return twist_; }

    void setVector(const Vector6& vector6) { twist_ = vector6; }

private:
    Vector6 twist_;
};

}  // namespace tpl

// convenience typedef (required)
typedef tpl::RigidBodyVelocities<double> RigidBodyVelocities;

} /* namespace rbd */
} /* namespace ct */
