/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace ct {
namespace core {
namespace internal {

//! Trait defining basic math functions for CppAD Auto-Diff types
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
    inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
    //! Solves a linear system of equations using Eigen's inverse functionality
    template <int Dims>
    inline static Eigen::Matrix<Scalar, Dims, 1> solve(const Eigen::Matrix<Scalar, Dims, Dims>& A,
        const Eigen::Matrix<Scalar, Dims, 1>& b)
    {
        return A.inverse() * b;
    }
};
}  // namespace internal
}  // namespace core
}  // namespace ct
