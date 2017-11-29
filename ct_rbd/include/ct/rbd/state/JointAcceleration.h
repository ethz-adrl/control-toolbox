/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {
namespace tpl {

/*!
 * \ingroup State
 *
 * \brief joint acceleration
 */
template <size_t NJOINTS, typename SCALAR>
class JointAcceleration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum NDOFS
    {
        NDOFS = NJOINTS
    };

    JointAcceleration() { setZero(); }
    JointAcceleration(const Eigen::Matrix<SCALAR, NJOINTS, 1>& acceleration) : acceleration_(acceleration) {}
    /// @brief get number of degrees of freedom
    size_t getSize() { return NJOINTS; }
    /// @brief get i-th joint acceleration
    SCALAR& operator()(size_t i) { return acceleration_(i); }
    /// @brief get joint acceleration
    Eigen::Matrix<SCALAR, NJOINTS, 1>& getAcceleration() { return acceleration_; }
    /// @brief get constant joint acceleration
    const Eigen::Matrix<SCALAR, NJOINTS, 1>& getAcceleration() const { return acceleration_; }
    /// @brief set joint acceleration
    void setAcceleration(const Eigen::Matrix<SCALAR, NJOINTS, 1>& acceleration) { acceleration_ = acceleration; }
    /// @brief set joint acceleration to zero
    void setZero() { acceleration_.setZero(); }
    static JointAcceleration<NJOINTS, SCALAR> Zero()
    {
        JointAcceleration<NJOINTS, SCALAR> jstate;
        jstate.setZero();
        return jstate;
    }

protected:
    Eigen::Matrix<SCALAR, NJOINTS, 1> acceleration_;
};

}  // namespace tpl

template <size_t NJOINTS>
using JointAcceleration = tpl::JointAcceleration<NJOINTS, double>;

}  // namespace rbd
}  // namespace ct
