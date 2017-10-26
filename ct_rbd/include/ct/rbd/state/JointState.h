/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

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
