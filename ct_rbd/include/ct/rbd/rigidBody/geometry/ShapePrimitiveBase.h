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

#ifndef SHAPEPRIMITIVE_H_
#define SHAPEPRIMITIVE_H_

#include <boost/concept_check.hpp>
#include <memory>

#include <Eigen/Dense>

#include <rigidBody/state/FloatingBaseState.h>
#include <rigidBody/geometry/BoundingBox.h>

namespace ct {
namespace rigidBody {
namespace geometry {

/// @brief Appearance primitive type
enum SHAPEPRIMITIVE_TYPE {UNKNOWN_SHAPEPRIMITIVE = 0, BOX = 1, SPHERE = 2, CYLINDER = 3, CAPSULE = 4, CONE = 5, PLANE = 6};

/**
 * @class ShapePrimitiveBase
 *
 * @brief primitive shape for objects
 */
class ShapePrimitiveBase
{
public:
	typedef std::shared_ptr<ShapePrimitiveBase> Ptr;

	ShapePrimitiveBase() {}

	virtual ShapePrimitiveBase::Ptr clone() const = 0;

	/// @brief get appearance primitive type
	virtual SHAPEPRIMITIVE_TYPE getShapePrimitiveType() { return UNKNOWN_SHAPEPRIMITIVE; };

	/// @brief compute bounding box
	virtual rigidBody::geometry::BoundingBox computeBoundingBox(const rigidBody::state::FloatingBaseState& transform) = 0;

	virtual ~ShapePrimitiveBase() {}


    /// @brief This is a cast that allows to down-cast to a derived class when passing a base class pointer
    template<class T>
    T* as(void)
    {
        BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ShapePrimitiveBase*>));
        return static_cast<T*>(this);
    }

    /// @brief This is a cast that allows to down-cast to a derived class when passing a base class pointer
    template<class T>
    const T* as(void) const
    {
        BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ShapePrimitiveBase*>));
        return static_cast<const T*>(this);
    }

protected:

};

template <typename GEOMETRIC_TYPE>
class ShapePrimitive : public ShapePrimitiveBase
{
public:
	rigidBody::geometry::BoundingBox computeBoundingBox(const rigidBody::state::FloatingBaseState& transform) override
	{
		return computeBoundingBoxImplementation(transform);
	}

	ShapePrimitiveBase::Ptr clone() const override {

		return ShapePrimitiveBase::Ptr(new GEOMETRIC_TYPE(static_cast<const GEOMETRIC_TYPE&>(*this)));
	}

protected:
	virtual rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) = 0;

};

/**
 * @class Box
 *
 * @brief appearance primitive: Box
 */
class Box: public ShapePrimitive<Box>
{
public:
	Box(double x, double y, double z):
	_x(x), _y(y), _z(z)  //length, width, height
	{}

	Box(const Box& box):
		_x(box._x),
		_y(box._y),
		_z(box._z)
	{
	}

	/// @brief get appearance primitive type box
	SHAPEPRIMITIVE_TYPE getShapePrimitiveType() {return BOX; }

	/// @brief get length
	double x() {return _x; }
	/// @brief get width
	double y() {return _y; }
	/// @brief get height
	double z() {return _z; }

	/// @brief compute bounding box
	rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) override {rigidBody::geometry::BoundingBox bbox; return bbox; }

	~Box() {};

protected:
	double _x, _y, _z;

};


/**
 * @class Sphere
 *
 * @brief appearance primitive: Sphere
 */
class Sphere: public ShapePrimitive<Sphere>
{
public:
	Sphere(double radius):
	_radius(radius)
	{}

	Sphere(const Sphere& sphere):
		_radius(sphere._radius)
	{
	}

	/// @brief get appearance primitive type sphere
	SHAPEPRIMITIVE_TYPE getShapePrimitiveType() {return SPHERE; }

	/// @brief get radius
	double radius() {return _radius; }

	/// @brief compute bounding box
	rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) override {rigidBody::geometry::BoundingBox bbox; return bbox; }

	~Sphere() {};

protected:
	double _radius;

};


/**
 * @class Capsule
 *
 * @brief appearance primitive: Capsule
 */
class Capsule: public ShapePrimitive<Capsule>
{
public:
	Capsule(double radius, double z_length):
		_radius(radius), _z_length(z_length)
	{}

	Capsule(const Capsule& capsule):
		_radius(capsule._radius),
		_z_length(capsule._z_length)
	{
	}

	/// @brief get appearance primitive type capsule
	SHAPEPRIMITIVE_TYPE getShapePrimitiveType() {return CAPSULE; }

	/// @brief get radius
	double radius() {return _radius; }
	/// @brief get length
	double z_length() {return _z_length; }

	/// @brief compute bounding box
	rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) override {rigidBody::geometry::BoundingBox bbox; return bbox; }

	~Capsule() {};

protected:
	double _radius;
	double _z_length; // length along z axis

};


/**
 * @class Cone
 *
 * @brief appearance primitive: Cone
 */
class Cone: public ShapePrimitive<Cone>
{
public:
	Cone(double radius, double z_length):
		_radius(radius), _z_length(z_length)
	{}

	Cone(const Cone& cone) :
		_radius(cone._radius),
		_z_length(cone._z_length)
	{}

	/// @brief get appearance primitive type cone
	SHAPEPRIMITIVE_TYPE getShapePrimitiveType() {return CONE; }

	/// @brief get radius
	double radius() {return _radius; }
	/// @brief get height
	double z_length() {return _z_length; }

	/// @brief compute bounding box
	rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) override {rigidBody::geometry::BoundingBox bbox; return bbox; }

	~Cone() {};

protected:
	double _radius;
	double _z_length; // length along z axis
};


/**
 * @class Cylinder
 *
 * @brief appearance primitive: Cylinder
 */
class Cylinder: public ShapePrimitive<Cylinder>
{
public:
	Cylinder(double radius, double z_length):
		_radius(radius), _z_length(z_length)
	{}

	Cylinder(const Cylinder& cylinder) :
		_radius(cylinder._radius),
		_z_length(cylinder._z_length)
	{}

	/// @brief get appearance primitive type cylinder
	SHAPEPRIMITIVE_TYPE getShapePrimitiveType() {return CYLINDER; }

	/// @brief get radius
	double radius() {return _radius; }
	/// @brief get height
	double z_length() {return _z_length; }

	/// @brief compute bounding box
	rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) override {rigidBody::geometry::BoundingBox bbox; return bbox; }

	~Cylinder() {};

protected:
	double _radius;
	double _z_length; // length along z axis
};


/**
 * @class Plane
 *
 * @brief appearance primitive: Plane
 */
class Plane: public ShapePrimitive<Plane>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Plane(Eigen::Matrix<double, 3, 1> normal, double offset):
		_normal(normal), _offset(offset)
	{}

	Plane(const Plane& plane):
		_normal(plane._normal),
		_offset(plane._offset)
	{}

	/// @brief get appearance primitive type plane
	SHAPEPRIMITIVE_TYPE getShapePrimitiveType() {return PLANE; }

	/// @brief get normal vector
	Eigen::Matrix<double, 3, 1> normal() {return _normal; }
	/// @brief get offset
	double offset() {return _offset; }

	/// @brief compute bounding box
	rigidBody::geometry::BoundingBox computeBoundingBoxImplementation(const rigidBody::state::FloatingBaseState& transform) override {rigidBody::geometry::BoundingBox bbox; return bbox; }

	~Plane() {};

protected:
	Eigen::Matrix<double, 3, 1> _normal;
	double _offset;
};

} // namespace geometry
} // namespace rigidBody
} // namespace ct

#endif /* SHAPEPRIMITIVE_H_ */
