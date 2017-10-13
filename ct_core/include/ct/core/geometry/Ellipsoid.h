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
namespace tpl {


template<typename SCALAR>
class Ellipsoid
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;
    typedef Eigen::Matrix<SCALAR, 3, 3> Matrix3s;
    typedef Eigen::Quaternion<SCALAR> Quaternions;

    /**
     * @brief      Default constructor
     *
     * @param[in]  x0    The center of the ellipsoid in 3d space
     * @param[in]  A     The matrix of half axes
     * @param[in]  S     The orientation of the ellipsoid
     */
    Ellipsoid(
        const Vector3s& x0,
        const Matrix3s& A,
        const Matrix3s& S)
    :
        x0_(x0),
        A_(A),
        S_(S)
    {}

    void setFromQuaterion(const Quaternions& quat)
    {
        S_ = Matrix3s(quat);
    }

    /**
     * @brief      Returns a value =< 1 if point x is inside the ellpsoid, > 1
     *             otherwise
     *
     * @param[in]  x     The point to be checked
     *
     * @return     { description_of_the_return_value }
     */
    SCALAR insideEllipsoid(const Vector3s& x)
    {
        return (x - x0_).transpose() * S_ * A_.transpose() * A_ * S_.transpose() * (x - x0_) - SCALAR(1.0);
    }

    const Vector3s& x0() const {return x0_; }
    Vector3s& x0() {return x0_; }

    const Matrix3s& A() const {return A_; }
    Matrix3s& A() {return A_; }

    const Matrix3s& S() const {return S_; }
    Matrix3s& S() {return S_; }


private:
    Vector3s x0_; // The center of the ellipsoid
    Matrix3s A_; // The matrix of halfaxes
    Matrix3s S_; // The orientation of the ellipsoid

};

}

typedef tpl::Ellipsoid<double> Ellipsoid;

} // namespace optcon
} // namespace ct

