/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace rbd {

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

    /// @brief normalize the joint state to be in the range [lowerLimitVec, lowerLimitVec + 2pi)
    template <typename T>
    void toUniquePosition(T lowerLimitVec, double tolerance = 0.0)
    {
        assert(lowerLimitVec.size() == NJOINTS && "Wrong limit dimensions");
        for (size_t i = 0; i < NJOINTS; ++i)
        {
            // compute the integer of multiple of 2*PI to substract
            int k_lower = std::floor((getPosition(i) - (lowerLimitVec[i] - tolerance)) / (2 * M_PI));
            int k_upper = std::floor((getPosition(i) - (lowerLimitVec[i] + tolerance)) / (2 * M_PI));

            if (std::abs(k_lower) <= std::abs(k_upper))
            {
                if (k_lower != 0)
                    getPosition(i) -= k_lower * 2 * M_PI;
            }
            else if (k_upper != 0)
                getPosition(i) -= k_upper * 2 * M_PI;
        }
    }

    /// @brief check joint position limits assuming limits and joint position are in the same range e.g. [-pi, pi)
    template <typename T>
    bool checkPositionLimits(const T lowerLimit, const T upperLimit, const double tolerance = 0.0)
    {
        assert(lowerLimit.size() == NJOINTS && upperLimit.size() == NJOINTS && "Wrong limit dimensions");
        for (size_t i = 0; i < NJOINTS; i++)
        {
            if ((std::abs(getPosition(i) - lowerLimit[i]) > tolerance && getPosition(i) < lowerLimit[i]) ||
                (std::abs(getPosition(i) - upperLimit[i]) > tolerance && getPosition(i) > upperLimit[i]))
            {
                return false;
            }
        }
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
            if (std::abs(getVelocity(i)) > limit[i])
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

}  // namespace rbd
}  // namespace ct
