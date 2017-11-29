/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace ct {
namespace rbd {
namespace internal {

/*!
 * \brief Trait providing basic mathematical operations for CppAD datatypes
 */
struct CppADDoubleTrait
{
    typedef CppAD::AD<double> Scalar;

    inline static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar tan(const Scalar& x) { return CppAD::tan(x); }
    inline static Scalar sinh(const Scalar& x) { return CppAD::sinh(x); }
    inline static Scalar cosh(const Scalar& x) { return CppAD::cosh(x); }
    inline static Scalar tanh(const Scalar& x) { return CppAD::tanh(x); }
    inline static Scalar exp(const Scalar& x) { return CppAD::exp(x); }
    inline static Scalar fabs(const Scalar& x) { return CppAD::fabs(x); }
    template <int Dims>
    inline static Eigen::Matrix<Scalar, Dims, 1> solve(const Eigen::Matrix<Scalar, Dims, Dims>& A,
        const Eigen::Matrix<Scalar, Dims, 1>& b)
    {
        return A.inverse() * b;
    }
};
}
}
}
