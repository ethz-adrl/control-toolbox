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

#include <iostream>

#include <rigidBody/state/FloatingBaseState.h>

#include <physics/contacts/SoftContactModel.h>

using namespace std;
namespace ct {
namespace physics {
namespace contacts {


void SoftContactModel::calculateContactForce(const rigidBody::state::FloatingBaseState& state1, const rigidBody::state::FloatingBaseState& state2, const collisions::Collision& collision, ContactForce& contactForce)
{
	collision_id_t id = 0;// TODO FIX ME: = getCollisionId(collision);
	//collision_id_t id = getCollisionId(collision);

	ContactState::displacement_t displacementWF;
	ContactState::displacementVelocity_t displacementVelocityWF;

	ContactForce contactForceWF;

	calculateDisplacement(state1, state2, collision, _contactState[id], displacementWF, displacementVelocityWF);

	_contactDynamics->calculateStictionForce(displacementWF, displacementVelocityWF, _contactState[id], contactForceWF);

	if (_contactState[id].isSliding() || _contactState[id].getAngle() > _parameters.frictionConeAngle)
	{
		_contactDynamics->calculateFrictionForce(displacementWF, displacementVelocityWF, _contactState[id], contactForce);

		_contactState[id].setTouchDownWorld(collision.getContactPoint());
		//resetCollisionPointonBody

//		if (displacementVelocityWF.getTangential().norm() < _parameters.slidingVelocityThreshold)
//		{
//			_contactState[id].setSliding(false);
//		} else
//		{
//			_contactState[id].setSliding(true);
//		}
	}

	//transformContactForceToWorldFrame();
	std::cout << contactForceWF.getForceW() << std::endl;
}


void SoftContactModel::calculateContactForces(std::shared_ptr<world::RigidBodyHandler> objectHandler, const collisions::Collisions& collisions, ContactForces& contactForces)
{
	std::cout << "SoftContactModel::calculateContactForces: number of collisions: " << collisions.size() << std::endl;
	for(size_t i=0; i<collisions.size(); i++)
	{
		// Initialization
		rigidBody::BodyId id1, id2;
		rigidBodyGroup::GroupId idgroup1, idgroup2;
		collisions[i].getObjectIds(id1, id2);
		collisions[i].getGroupIds( idgroup1, idgroup2);
		ContactForce contactforce_tmp;

		rigidBody::state::FloatingBaseState state1 = objectHandler->getGroup(idgroup1)->getObject(id1)->getState();
		rigidBody::state::FloatingBaseState state2 = objectHandler->getGroup(idgroup2)->getObject(id2)->getState();

		// Calculate contact force
		contactforce_tmp.setObjectId(id1);
		contactforce_tmp.setGroupId(idgroup1);
		calculateContactForce(state1, state2, collisions[i], contactforce_tmp);

		contactForces.push_back(contactforce_tmp);


	}
}


void SoftContactModel::calculateDisplacement(
		const rigidBody::state::FloatingBaseState& state1,
		const rigidBody::state::FloatingBaseState& state2,
		const collisions::Collision& collision,
		const contacts::ContactState& contactState,
		ContactState::displacement_t& displacementWF,
		ContactState::displacementVelocity_t& displacementVelocityWF
	)
{
	// get the transformation of object 1
	const kindr::RotationQuaternionPD& R_O1_W = state1.getOrientation();

	// get contact point of object 1 in its frame
	kindr::Position3D O1_r_O10_P(contactState.getContactPointObject1OF());

	// get contact point of object 1 in world coordinates
	kindr::Position3D W_r_O10_P =  R_O1_W.inverseRotate(O1_r_O10_P);
	const kindr::Position3D W_r_W0_O10 = state1.getPosition();
	Eigen::Vector3d W_r_W0_P = (W_r_W0_O10 + W_r_O10_P).toImplementation();

	// calculate the displacement in the world frame
	displacementWF = W_r_W0_P - contactState.getTouchDownWorld();

	displacementVelocityWF = state1.getTranslationalVelocity().toImplementation() + W_r_W0_P.cross(state1.getLocalAngularVelocity().toImplementation());
}




} // namespace contacts
} // namespace physics
} // namespace ct

