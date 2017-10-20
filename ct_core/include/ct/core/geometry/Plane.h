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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
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
