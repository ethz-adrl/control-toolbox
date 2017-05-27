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

#include <physics/collisions/SimpleCollisionChecker.h>


namespace ct {
namespace physics {
namespace collisions {


bool SimpleCollisionChecker::checkCollisionsBetweenTwoObjects(
		rigidBody::RigidBody::Ptr object1,
		rigidBody::RigidBody::Ptr object2,
		Collisions& collisions	)
{
	if( _broadphaseCollisionChecker->checkCollisionsBetweenTwoObjects(object1, object2) ){

		if( _narrowphaseCollisionChecker->checkCollisionsBetweenTwoObjects(object1, object2, collisions)){
			return true;
		}
	}
	return false;
}


bool SimpleCollisionChecker::checkSelfCollision( rigidBodyGroup::RigidBodyGroup::Ptr group, Collisions & collisions)
{
	if(group->getNumberOfObjects() < 2 ) return false;

	for (rigidBodyGroup::RigidBodyGroup::BodyMap::const_iterator it1=group->getObjects().begin(); it1!=group->getObjects().end(); ++it1)
	{
		rigidBodyGroup::RigidBodyGroup::BodyMap::const_iterator it1_next=it1;
		it1_next++;
		for (rigidBodyGroup::RigidBodyGroup::BodyMap::const_iterator it2=it1_next; it2!=group->getObjects().end(); ++it2)
		{
			// Specify for object 1
			Collisions collisions_tmp1;
			if( checkCollisionsBetweenTwoObjects(it1->second, it2->second, collisions_tmp1) )
			{
				for(size_t i=0; i < collisions_tmp1.size(); i++)
				{
					collisions_tmp1[i].setGroupIds(group->getGroupId(), group->getGroupId());
					collisions.push_back(collisions_tmp1[i]);
				}
			}
			// Specify for object 2
			Collisions collisions_tmp2;
			if( checkCollisionsBetweenTwoObjects(it2->second, it1->second, collisions_tmp2) )
			{
				for(size_t i=0; i < collisions_tmp2.size(); i++)
				{
					collisions_tmp2[i].setGroupIds(group->getGroupId(), group->getGroupId());
					collisions.push_back(collisions_tmp2[i]);
				}
			}
		}
	}

	if( collisions.size() > 0)
	{
		return true;
	}
	return false;
}



bool SimpleCollisionChecker::checkCollisionsBetweenTwoGroups(
		rigidBodyGroup::RigidBodyGroup::Ptr group1,
		rigidBodyGroup::RigidBodyGroup::Ptr group2,
		Collisions & collisions)
{
	if( _broadphaseCollisionChecker->checkCollisionsBetweenTwoGroups(group1, group2) )
	{
		for (rigidBodyGroup::RigidBodyGroup::BodyMap::const_iterator it1=group1->getObjects().begin(); it1!=group1->getObjects().end(); ++it1)
		{
			for (rigidBodyGroup::RigidBodyGroup::BodyMap::const_iterator it2=group2->getObjects().begin(); it2!=group2->getObjects().end(); ++it2)
			{
				//important:keep two-way collision checking (collisions are defined via the first impact surface)
				// Specify for object 1
				Collisions collisions_tmp1;
				if( checkCollisionsBetweenTwoObjects(it1->second, it2->second, collisions_tmp1) )
				{
					for(size_t i=0; i < collisions_tmp1.size(); i++)
					{
						collisions_tmp1[i].setGroupIds(group1->getGroupId(), group2->getGroupId());
						collisions.push_back(collisions_tmp1[i]);
					}
				}

				// Specify for object 2
				Collisions collisions_tmp2;
				if( checkCollisionsBetweenTwoObjects(it2->second, it1->second, collisions_tmp2) )
				{
					for(size_t i=0; i < collisions_tmp2.size(); i++)
					{
						collisions_tmp2[i].setGroupIds(group2->getGroupId(), group1->getGroupId());
						collisions.push_back(collisions_tmp2[i]);
					}
				}
			}
		}
	}

	if( collisions.size() > 0)
	{
		return true;
	}
	return false;
}




// main collision checking function
// - checks collisions between all groups
// - checks self collision
bool SimpleCollisionChecker::checkCollisions(std::shared_ptr<world::RigidBodyHandler> objectHandler, Collisions& collisions )
{
	// check collisions between the different groups
	for (world::GroupMap::const_iterator it1=objectHandler->getGroups().begin(); it1!=objectHandler->getGroups().end(); ++it1)
	{
		world::GroupMap::const_iterator it1_next=it1;
		it1_next++;
		for (world::GroupMap::const_iterator it2=it1_next; it2!=objectHandler->getGroups().end(); ++it2)
		{
			Collisions collisions_tmp;
			if( _broadphaseCollisionChecker->checkCollisionsBetweenTwoGroups(it1->second, it2->second) )
			{
				if(checkCollisionsBetweenTwoGroups(it1->second, it2->second, collisions_tmp))
				{
					for(size_t i=0; i < collisions_tmp.size(); i++)
					{
						collisions.push_back(collisions_tmp[i]);
					}
				}
			}
		}
	}

	// check self collision at each group
	if(isSelfCollisionCheckingActive()){
		for (world::GroupMap::const_iterator it=objectHandler->getGroups().begin(); it!=objectHandler->getGroups().end(); ++it)
		{
			Collisions collisions_tmp;
			if(checkSelfCollision(it->second, collisions_tmp) )
			{
				for(size_t i=0; i < collisions_tmp.size(); i++)
				{
					collisions.push_back(collisions_tmp[i]);
				}
			}
		}
	}

	if( collisions.size() > 0)
	{
		return true;
	}
	return false;
}


} // namespace collisions
} // namespace physics
} // namespace ct



