/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {
namespace tpl {

/**
 * @class RigidBodyAcceleration
 *
 * \ingroup State
 *
 * @brief acceleration of a rigid body
 */
template <typename SCALAR>
class RigidBodyAcceleration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef kindr::Acceleration<SCALAR, 3> LinearAcceleration;
    typedef kindr::AngularAcceleration<SCALAR, 3> AngularAcceleration;

    RigidBodyAcceleration() { setZero(); }
    RigidBodyAcceleration(LinearAcceleration& transAcceleration, AngularAcceleration& anglAcceleration)
        : anglAcceleration_(anglAcceleration), transAcceleration_(transAcceleration)
    {
    }

    RigidBodyAcceleration(const Eigen::Matrix<SCALAR, 6, 1>& in) { fromVector6d(in); }
    void fromVector6d(const Eigen::Matrix<SCALAR, 6, 1>& in)
    {
        anglAcceleration_ << in.template head<3>();
        transAcceleration_ << in.template tail<3>();
    }

    const Eigen::Matrix<SCALAR, 6, 1> getVector6d() const
    {
        Eigen::Matrix<SCALAR, 6, 1> vector6;
        vector6 << anglAcceleration_.toImplementation(), transAcceleration_.toImplementation();
        return vector6;
    }

    /// @brief get translatory acceleration
    LinearAcceleration& getTranslationalAcceleration() { return transAcceleration_; }
    /// @brief get constant translatory acceleration
    const LinearAcceleration& getTranslationalAcceleration() const { return transAcceleration_; }
    /// @brief get angular acceleration
    AngularAcceleration& getAngularAcceleration() { return anglAcceleration_; }
    /// @brief get constant angular acceleration
    const AngularAcceleration& getAngularAcceleration() const { return anglAcceleration_; }
    /// @brief get accelerations as a "Twist"
    Eigen::Matrix<SCALAR, 6, 1> accelerations()
    {
        Eigen::Matrix<SCALAR, 6, 1> out;
        out << anglAcceleration_.toImplementation(), transAcceleration_.toImplementation();
        return out;
    }

    /// @brief set translatory acceleration
    void setTranslatoryAcceleration(LinearAcceleration& transAcceleration) { transAcceleration_ = transAcceleration; }
    /// @brief set angular acceleration
    void setAngularAcceleration(AngularAcceleration& anglAcceleration) { anglAcceleration_ = anglAcceleration; }
    /// @brief set acceleration to zero
    void setZero()
    {
        // cannot use .setZero() from members here due to codegen
        anglAcceleration_ = AngularAcceleration(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0));
        transAcceleration_ = LinearAcceleration(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0));
    }

    static RigidBodyAcceleration Zero()
    {
        RigidBodyAcceleration state;
        state.setZero();
        return state;
    }

protected:
    AngularAcceleration anglAcceleration_;
    LinearAcceleration transAcceleration_;
};

}  // namespace tpl

// convenience typedef (required)
typedef tpl::RigidBodyAcceleration<double> RigidBodyAcceleration;

}  // namespace rbd
}  // namespace ct
