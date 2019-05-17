/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

/*!
 * \ingroup State
 *
 * \brief joint acceleration
 */
template <size_t NJOINTS, typename SCALAR = double>
class JointAcceleration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum NDOFS
    {
        NDOFS = NJOINTS
    };

    typedef Eigen::Matrix<SCALAR, NJOINTS, 1> Acceleration;

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
    Acceleration acceleration_;
};

}  // namespace rbd
}  // namespace ct
