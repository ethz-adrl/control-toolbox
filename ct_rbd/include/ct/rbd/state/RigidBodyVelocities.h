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

typedef tpl::RigidBodyVelocities<double> RigidBodyVelocities;

} /* namespace rbd */
} /* namespace ct */
