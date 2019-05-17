/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

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

    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> point_measurements_t;

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
        d.setOnes();  // d normalized to 1

        for (size_t i = 0; i < points.size(); i++)
        {
            A.row(i) = points[i];
        }

        Eigen::Matrix<double, 4, 1> coefficients;
        coefficients.head<3>() = A.colPivHouseholderQr().solve(d);
        coefficients(3) = 1.0;  // d normalized to 1

        return Plane(coefficients);
    }

private:
};

}  // namespace core
}  // namespace ct

#pragma once
