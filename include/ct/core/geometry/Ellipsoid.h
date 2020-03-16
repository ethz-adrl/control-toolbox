/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace tpl {


template <typename SCALAR>
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
    Ellipsoid(const Vector3s& x0, const Matrix3s& A, const Matrix3s& S) : x0_(x0), A_(A), S_(S) {}
    void setFromQuaterion(const Quaternions& quat) { S_ = Matrix3s(quat); }
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

    const Vector3s& x0() const { return x0_; }
    Vector3s& x0() { return x0_; }
    const Matrix3s& A() const { return A_; }
    Matrix3s& A() { return A_; }
    const Matrix3s& S() const { return S_; }
    Matrix3s& S() { return S_; }
private:
    Vector3s x0_;  // The center of the ellipsoid
    Matrix3s A_;   // The matrix of halfaxes
    Matrix3s S_;   // The orientation of the ellipsoid
};
}

typedef tpl::Ellipsoid<double> Ellipsoid;

}  // namespace optcon
}  // namespace ct
