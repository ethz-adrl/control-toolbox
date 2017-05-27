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

#ifndef OBJECTBASE_HPP_
#define OBJECTBASE_HPP_

#include <boost/concept_check.hpp>
#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <list>


#include <rigidBody/geometry/BoundingBox.h>
#include <rigidBody/state/FloatingBaseState.h>
#include <rigidBody/geometry/AppearanceBase.h>
#include <rigidBody/geometry/ShapePrimitiveBase.h>

namespace ct {
namespace rigidBody {

/// @brief object ID
typedef int BodyId;

/**
 * @class ObjectBase
 *
 * @brief physical bodies in the simulation have to be defined as objects
 *
 * objects: links of robots, surfaces, obstacles
 *
 * Changlog: Markus, 11.04.15 - uncommented all bounding box functions - need to be adapted for multiple appearances per rigid body
 */
class RigidBody
{
public:
	typedef std::shared_ptr<RigidBody> Ptr;

	RigidBody (const std::string& name = std::string("unnamed")) :
		bodyId_(-1),
		name_(name)
	{
		assert("Not to be used");
	}

  //todo: ask michael if this is the correct way to do it.
	RigidBody (std::shared_ptr<geometry::AppearanceBase> appearance, const std::string& name = std::string("unnamed")) :
		bodyId_(-1),
		name_(name)
	{
		appearance_.push_back(appearance);
		//computeBoundingBox(); //todo: uncomment
	}

	RigidBody (std::vector<std::shared_ptr<geometry::AppearanceBase> > appearance, const std::string& name = std::string("unnamed")) :
		bodyId_(-1),
		appearance_(),
		name_(name)
	{
		appearance_=appearance;
		//computeBoundingBox(); //todo: uncomment
	}

	  //todo: ask michael - how to get this working
	RigidBody (rigidBody::state::FloatingBaseState& state, std::shared_ptr<geometry::AppearanceBase> appearance, const std::string& name = std::string("unnamed")) :
		bodyId_(-1),
		appearance_(),
		state_(state),
		name_(name)
	{
		appearance_.push_back(appearance);
		//computeBoundingBox();
	}

	RigidBody (rigidBody::state::FloatingBaseState& state, std::vector<std::shared_ptr<geometry::AppearanceBase> > appearance, const std::string& name = std::string("unnamed")) :
		bodyId_(-1),
		appearance_(appearance),
		state_(state),
		name_(name)
	{
		appearance_=appearance;
		//computeBoundingBox();  //todo: uncomment
	}

	RigidBody(const RigidBody& rigidBody):
		bodyId_(rigidBody.bodyId_),
		boundingBox_(rigidBody.boundingBox_),
		state_(rigidBody.state_),
		name_(rigidBody.name_)
	{
		for (size_t i=0; i<rigidBody.appearance_.size(); i++)
		{
			appearance_.push_back(geometry::AppearanceBase::Ptr(new geometry::AppearanceBase(*rigidBody.appearance_[i])));
		}
	}

	~RigidBody () {};

	/// @brief get object ID
	BodyId getBodyId() {return bodyId_; }
	/// @brief set object ID
	void setBodyId(BodyId id) {bodyId_ = id; }

	///@brief get number of appearances for this rigid body, required e.g. for visualization and collision checking
	uint16_t getNumberOfAppearances() {return appearance_.size();}

	/// @brief get appearance of the object
	std::shared_ptr<geometry::AppearanceBase>& getAppearance(uint16_t index = 0) {return appearance_[index]; }  //markus: additional index with default 0;

	/// @brief get bounding box
	rigidBody::geometry::BoundingBox& getBoundingBox() {return boundingBox_; }

	/// @brief get object states
	rigidBody::state::FloatingBaseState& getState() {return state_; }
	/// @brief set object states
	void setState(const rigidBody::state::FloatingBaseState& state) {state_ = state; }


	/// @brief get world state of an appearance of this object.
	void getAppearanceStateWorld(uint16_t appearanceId, rigidBody::state::FloatingBaseState& baseState) {
		assert(appearanceId<appearance_.size() && "AppearanceID out of bounds.");
		Eigen::Matrix<double, 3, 3> R_ObjectComtoAppearanceCenter = appearance_[appearanceId]->transformationComGeomcenter().getRotation().toImplementation().toRotationMatrix();
		Eigen::Matrix<double, 3, 3> R_WorldToObjectCom = state_.getOrientation().toImplementation().toRotationMatrix();
		Eigen::Matrix<double, 3, 3> R_WorldToApperanceCenter = R_WorldToObjectCom * R_ObjectComtoAppearanceCenter;
		baseState.getOrientation().toImplementation() = Eigen::Quaternion<double>(R_WorldToApperanceCenter);

		Eigen::Matrix<double, 3, 1> world_x_0_to_com = state_.getPosition().toImplementation(); // Origin to CoM expressed in world coordinates
		Eigen::Matrix<double, 3, 1> com_x_com_to_geomcenter = appearance_[appearanceId]->transformationComGeomcenter().getPosition().toImplementation(); // Geometric center expressed in link CoM coordinates
		baseState.getPosition().toImplementation() = world_x_0_to_com +  state_.getOrientation().inverseRotate(com_x_com_to_geomcenter); // Origin to geometric center expressed in world coordinates.
	}


	std::string& name() { return name_; }


	/// @brief compute bounding box
	//void computeBoundingBox(){ //todo: re-implement this
	//	boundingBox_ = appearance_->shape()->computeBoundingBox(state_); }

protected:
	BodyId bodyId_;

	std::vector<std::shared_ptr<geometry::AppearanceBase>> appearance_;
	rigidBody::geometry::BoundingBox boundingBox_;

	rigidBody::state::FloatingBaseState state_;

	std::string name_;
};

} // namespace rigidBody
} // namespace ct

#endif /* OBJECTBASE_HPP_ */
