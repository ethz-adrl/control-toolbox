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

#ifndef APPEARANCE_H_
#define APPEARANCE_H_

#include <common/Color.h>
#include <common/kinematicTypes.h>

#include <rigidBody/geometry/ShapePrimitiveBase.h>

namespace ct {
namespace rigidBody {
namespace geometry {


typedef uint16_t AppearanceId;

class ShapePrimitiveBase;


/**
 * @class AppearanceBase
 *
 * @brief contains all the primitive appearance of each link of a robot
 */
class AppearanceBase
{
public:
	typedef std::shared_ptr<AppearanceBase> Ptr;

	static common::kinematics::pose_t NoRotateComToGeomcenter(){
		return common::kinematics::pose_t(
				common::kinematics::pose_t::Position(0,0,0),
				common::kinematics::pose_t::Rotation(kindr::EulerAnglesXyzPD(0.0, 0.0, 0.0))
		);
	}

	/**
	 * Rotate the CoM reference frame by 90 degrees around y axis
	 * This swaps x and z axis, to shape is displayed CoM x axis.
	 * Use when axis of rotation does not correspond to link direction (FE-joints).
	 */
	static common::kinematics::pose_t RotateY90ComToGeomcenter(){
		return common::kinematics::pose_t(
				common::kinematics::pose_t::Position(0,0,0),
				common::kinematics::pose_t::Rotation(kindr::EulerAnglesXyzPD(0.0, M_PI/2.0, 0.0))
		);
	}

	/**
	 * Collection of attributes that specify the appearance of an object
	 *
	 * @param shape               the form (cylinder, cube, etc.) of the object
	 * @param color               the color of the above form
	 * @param T_com_geomcenter    Transformation from CoM frame to Geometric Center
	 */
	AppearanceBase(std::shared_ptr<geometry::ShapePrimitiveBase> shape, const common::Color& color = common::Color::BLUE,
			const common::kinematics::pose_t& T_com_geomcenter = NoRotateComToGeomcenter(), uint16_t appearanceID = 0, bool isColliding = true):
				shape_(shape),
				color_(color),
				T_com_geomcenter_(T_com_geomcenter),
				appearanceID_(appearanceID),
				isColliding_(isColliding)
	{
	}

	//Copy-constructor
	AppearanceBase(const AppearanceBase& appearance):
		shape_(appearance.shape_->clone()),
		color_(appearance.color_),
		T_com_geomcenter_(appearance.T_com_geomcenter_),
		appearanceID_(appearance.appearanceID_),
		isColliding_(appearance.isColliding_)
	{
	}

	virtual ~AppearanceBase() {}

	/// @brief get the primitive appearance of the link
	std::shared_ptr<geometry::ShapePrimitiveBase> shape() { return shape_; }
	common::Color& color() { return color_; }
	common::kinematics::pose_t& transformationComGeomcenter() { return T_com_geomcenter_; }

	/// @brief Return the identifier of this appearance
	AppearanceId getAppearanceId() const {return appearanceID_;}

	bool isColliding() const {return isColliding_;}

protected:
	std::shared_ptr<geometry::ShapePrimitiveBase> shape_;
	common::Color color_;
	common::kinematics::pose_t T_com_geomcenter_; ///< transformation from object CoM to its geometric center
	AppearanceId appearanceID_;
	bool isColliding_; //states if this appearance shall be checked for collisions

public:

};

typedef std::vector<AppearanceBase> Appearances;


} // namespace geometry
} // namespace rigidBody
} // namespace ct

#endif /* APPEARANCE_H_ */
