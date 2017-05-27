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

#include <physics/contacts/SpringDamperContactDynamics.h>

namespace ct {
namespace physics {
namespace contacts {

// all variables are expressed in the world frame (WF)
// the contact force is expressed in WF and describes the force that object 1 applies to object 2

void SpringDamperContactDynamics::calculateStictionForce(
		const ContactState::displacement_t& displacementWF,
		const ContactState::displacementVelocity_t& displacementVelocityWF,
		const ContactState& contactState,
		ContactForce& contactForceWF)
{
	double normalDisplacement = contactState.getContactNormalWorld().transpose() * displacementWF;
	Eigen::Vector3d tangentialDisplacement = displacementWF - normalDisplacement * contactState.getContactNormalWorld();

	double normalDisplacementVelocity = contactState.getContactNormalWorld().transpose() * displacementVelocityWF;
	Eigen::Vector3d tangentialDisplacementVelocity = displacementVelocityWF - normalDisplacementVelocity * contactState.getContactNormalWorld();

	Eigen::Vector3d F_normal = (calculateSpringForceNormal(normalDisplacement) + calculateDampingForceNormal(normalDisplacementVelocity))
			* contactState.getContactNormalWorld();

	Eigen::Vector3d F_tangential = calculateSpringForceTangential(tangentialDisplacement.norm()) * tangentialDisplacement.normalized() +
			calculateDampingForceTangential(tangentialDisplacementVelocity.norm()) * tangentialDisplacementVelocity.normalized();

	contactForceWF.setForceW(F_normal + F_tangential);
}

void SpringDamperContactDynamics::calculateFrictionForce(
		const ContactState::displacement_t& displacementWF,
		const ContactState::displacementVelocity_t& displacementVelocityWF,
		const ContactState& contactState,
		ContactForce& contactForceWF)
{
	double normalDisplacementVelocity = contactState.getContactNormalWorld().transpose() * displacementVelocityWF;
	Eigen::Vector3d tangentialDisplacementVelocity = displacementVelocityWF - normalDisplacementVelocity * contactState.getContactNormalWorld();

	double F_normal_norm = contactState.getContactNormalWorld().transpose() * contactForceWF.getForceW();
	Eigen::Vector3d F_normal = F_normal_norm * contactState.getContactNormalWorld();
	Eigen::Vector3d F_tangential = tangentialDisplacementVelocity.normalized() * (F_normal_norm * _parameters.mu);

	contactForceWF.setForceW(F_normal + F_tangential);
}


inline double SpringDamperContactDynamics::calculateSpringForceNormal(double displacement)
{
	return _parameters.k_normal*displacement;
}

inline double SpringDamperContactDynamics::calculateSpringForceTangential(double displacement)
{
	return _parameters.k_tangential*displacement;
}

inline double SpringDamperContactDynamics::calculateDampingForceNormal(double velocity)
{
	return _parameters.d_normal*velocity;
}

inline double SpringDamperContactDynamics::calculateDampingForceTangential(double velocity)
{
	return _parameters.d_tangential*velocity;
}

} // namespace contacts
} // namespace physics
} // namespace ct
