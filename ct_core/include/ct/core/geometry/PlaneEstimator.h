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

#ifndef CT_CORE_PLANEESTIMATOR_H_
#define CT_CORE_PLANEESTIMATOR_H_

#include "Plane.h"

namespace ct {
namespace core {

//! Estimates a Plane from a number of 3D points using least squares
/*!
 * Given a set of measurements, a plane is fitted to them using least squares.
 */
class PlaneEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > point_measurements_t;

	//! constructor
	PlaneEstimator() {}

	//! destructor
	~PlaneEstimator() {}

	//! estimate the plane
	/*!
	 * Fits a plane to 3D points
	 *
	 * \warning Throws an exception if not at least three points are provided.
	 *
	 * @param points the ppints to fit to
	 * @return the estimated plane
	 */
	Plane estimate(const point_measurements_t& points)
	{
		if (points.size() < 3)
			throw std::runtime_error("Point measurement vector should contain at least 3 entries!");

		Eigen::MatrixXd A(points.size(), 3);
		Eigen::MatrixXd d(points.size(), 1);
		d.setOnes(); // d normalized to 1

		for (size_t i=0; i<points.size(); i++)
		{
			A.row(i) = points[i];
		}

		Eigen::Matrix<double, 4, 1> coefficients;
		coefficients.head<3>() = A.colPivHouseholderQr().solve(d);
		coefficients(3) = 1.0; // d normalized to 1

		return Plane(coefficients);
	}

private:

};

}  // namespace core
}  // namespace ct

#endif /* CT_CORE_PLANEESTIMATOR_H_ */
