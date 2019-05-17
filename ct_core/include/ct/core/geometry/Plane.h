/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

//! Implements a geometrical 3D plane of type \f$ ax + by + cz = d \f$
class Plane
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! default constructor
    Plane() { coefficients_.setZero(); }
    //! constructor
    /*!
	 * constructs a plane \f$ ax + by + cz = d \f$ where the coefficients
	 * are ordered as \f$ \begin{bmatrix} a & b & c & d \end{bmatrix} \f$
	 * @param coefficients plane coefficients
	 */
    Plane(const Eigen::Matrix<double, 4, 1>& coefficients) : coefficients_(coefficients) {}
    //! constructor
    /*!
	 * constructs a plane \f$ ax + by + cz = d \f$
	 * @param a coefficient a
	 * @param b coefficient b
	 * @param c coefficient c
	 * @param d coefficient d
	 */
    Plane(double a, double b, double c, double d) { coefficients_ << a, b, c, d; }
    //! returns the coefficients
    /*!
	 * returns the coefficients of the plane \f$ ax + by + cz = d \f$ in order
	 * \f$ \begin{bmatrix} a & b & c & d \end{bmatrix} \f$
	 * @return planecoefficients
	 */
    Eigen::Matrix<double, 4, 1>& getCoefficients() { return coefficients_; }
    //! returns a single coefficient
    /*!
	 * returns a single coefficient by index, where 0=a, 1=b, 2=c and 3=d
	 * @param i index, needs to be smaller than 4
	 * @return reference of the coefficient
	 */
    double& getCoefficient(size_t i)
    {
        if (i >= 4)
            throw std::runtime_error("Index out of range, should be max 3.");
        return coefficients_(i);
    }

    //! get a
    double a() { return coefficients_[AIdx]; }
    //! get b
    double b() { return coefficients_[BIdx]; }
    //! get c
    double c() { return coefficients_[CIdx]; }
    //! get d
    double d() { return coefficients_[DIdx]; }
    //! sets the plane
    void set(double a, double b, double c, double d) { coefficients_ << a, b, c, d; };
    //! solve for x
    /*!
	 * finds the x value for the plane \f$ ax + by + cz = d \f$ given y and z
	 * @param y y-value
	 * @param z z-value
	 * @return x-value
	 */
    double solveX(double y, double z) { return (d() - b() * y - c() * z) / a(); }
    //! solve for y
    /*!
	 * finds the y value for the plane \f$ ax + by + cz = d \f$ given x and z
	 * @param x x-value
	 * @param z z-value
	 * @return y-value
	 */
    double solveY(double x, double z) { return (d() - a() * x - c() * z) / b(); }
    //! solve for z
    /*!
	 * finds the z value for the plane \f$ ax + by + cz = d \f$ given x and y
	 * @param x x-value
	 * @param y y-value
	 * @return z-value
	 */
    double solveZ(double x, double y) { return (d() - a() * x - b() * y) / c(); }
private:
    enum
    {
        AIdx = 0,  //! index of coefficient a
        BIdx = 1,  //! index of coefficient b
        CIdx = 2,  //! index of coefficient c
        DIdx = 3   //! index of coefficient d
    };

    Eigen::Matrix<double, 4, 1> coefficients_;  //! plane coefficients
};

}  // namespace core
}  // namespace ct


#pragma once
