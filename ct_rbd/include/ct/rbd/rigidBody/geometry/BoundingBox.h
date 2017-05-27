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

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#include <Eigen/Dense>

namespace ct {
namespace rigidBody {
namespace geometry {

/**
 * @class BoundingBox
 *
 * @brief Bounding box is a primitive volume containing the whole object or object group
 */
class BoundingBox
{
public:
	BoundingBox():
		_x(0),
		_y(0),
		_z(0)
	{}

	BoundingBox(double x, double y, double z) :
		_x(x),
		_y(y),
		_z(z)
	{}

	BoundingBox(const BoundingBox& boundingBox):
		_x(boundingBox._x),
		_y(boundingBox._y),
		_z(boundingBox._z)
	{}

	/// @brief get length
	double& x() { return _x; }
	/// @brief get width
	double& y() { return _y; }
	/// @brief get height
	double& z() { return _z; }

private:
	double _x;
	double _y;
	double _z;
};


} // namespace geometry
} // namespace rigidBody
} // namespace ct

#endif /* BOUNDINGBOX_H_ */
