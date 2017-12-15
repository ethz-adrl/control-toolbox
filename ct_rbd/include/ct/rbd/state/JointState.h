/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace rbd {
namespace tpl {

/**
 * @class JointState
 *
 * \ingroup State
 *
 * @brief joint state and joint velocity
 */
template <size_t NJOINTS, typename SCALAR = double>
class JointState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum NDOFS
    {
        NDOFS = NJOINTS
    };

    typedef Eigen::Matrix<SCALAR, 2 * NJOINTS, 1> joint_state_vector_t;
    typedef Eigen::Matrix<SCALAR, NJOINTS, 1> Position;
    typedef Eigen::Matrix<SCALAR, NJOINTS, 1> Velocity;

    typedef Eigen::VectorBlock<joint_state_vector_t, NJOINTS> JointPositionBlock;
    typedef Eigen::VectorBlock<const joint_state_vector_t, NJOINTS> JointPositionBlockConst;

    JointState() { setZero(); }
    JointState(const joint_state_vector_t& state) : state_(state) {}
    inline bool operator==(const JointState& rhs) const { return (this->state_ == rhs.state_); }
    inline bool operator!=(const JointState<NJOINTS, SCALAR>& rhs) const { return (this->state_ != rhs.state_); }
    inline JointState<NJOINTS, SCALAR> operator+(const JointState<NJOINTS, SCALAR>& rhs) const
    {
        return JointState<NJOINTS, SCALAR>(this->state_ + rhs.state_);
    }
    inline JointState<NJOINTS, SCALAR> operator-(const JointState<NJOINTS, SCALAR>& rhs) const
    {
        return JointState<NJOINTS, SCALAR>(this->state_ - rhs.state_);
    }

    inline JointState<NJOINTS, SCALAR> operator*(const SCALAR& scalar) const
    {
        return JointState<NJOINTS, SCALAR>(this->state_ * scalar);
    }
    inline JointState<NJOINTS, SCALAR> operator/(const SCALAR& scalar) const
    {
        return JointState<NJOINTS, SCALAR>(this->state_ / scalar);
    }

    inline bool operator<(const JointState<NJOINTS, SCALAR>& rhs) const
    {
        return (this->array() < rhs.array()).isConstant(1);
    }

    inline bool operator>(const JointState<NJOINTS, SCALAR>& rhs) const
    {
        return (this->array() > rhs.array()).isConstant(1);
    }

    bool isApprox(const JointState<NJOINTS, SCALAR>& rhs, const SCALAR& tol = 1e-10)
    {
        return state_.isApprox(rhs.state_, tol);
    }

    /// @brief get joint state
    JointPositionBlock getPositions() { return state_.template head<NJOINTS>(); }
    /// @brief get constant joint state
    const JointPositionBlockConst getPositions() const { return state_.template head<NJOINTS>(); }
    SCALAR& getPosition(size_t i)
    {
        assert(i < NJOINTS && "Invalid joint index");
        return state_(i);
    }
    const SCALAR& getPosition(size_t i) const
    {
        assert(i < NJOINTS && "Invalid joint index");
        return state_(i);
    }
    /// @brief check joint position limits
    template <typename T>
    bool checkPositionLimits(T lowerLimit, T upperLimit)
    {
        assert(lowerLimit.size() == NJOINTS && upperLimit.size() == NJOINTS && "Wrong limit dimensions");
        for (size_t i = 0; i < NJOINTS; ++i)
            if (getPosition(i) < lowerLimit[i] || getPosition(i) > upperLimit[i])
                return false;
        return true;
    }

    /// @brief get joint velocity
    JointPositionBlock getVelocities() { return state_.template tail<NJOINTS>(); }
    /// @brief get constant joint velocity
    const JointPositionBlockConst getVelocities() const { return state_.template tail<NJOINTS>(); }
    SCALAR& getVelocity(size_t i)
    {
        assert(i < NJOINTS && "Invalid joint index");
        return state_(NJOINTS + i);
    }
    const SCALAR& getVelocity(size_t i) const
    {
        assert(i < NJOINTS && "Invalid joint index");
        return state_(NJOINTS + i);
    }
    /// @brief check joint velocity limits
    template <typename T>
    bool checkVelocityLimits(T limit)
    {
        assert(limit.size() == NJOINTS && "Wrong limit dimension");
        for (size_t i = 0; i < NJOINTS; ++i)
            if (abs(getVelocity(i)) > limit[i])
                return false;
        return true;
    }


    joint_state_vector_t& toImplementation() { return state_; }
    const joint_state_vector_t& toImplementation() const { return state_; }
    /// @brief set states to zero
    void setZero() { state_.setZero(); }
    void setRandom() { state_.setRandom(); }
protected:
    joint_state_vector_t state_;
};

}  // namespace tpl

template <size_t NJOINTS>
using JointState = tpl::JointState<NJOINTS, double>;

}  // namespace rbd
}  // namespace ct
