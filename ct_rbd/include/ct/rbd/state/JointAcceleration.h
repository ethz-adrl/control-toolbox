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
