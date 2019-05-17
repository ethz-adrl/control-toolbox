/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cppad/cg.hpp>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>


namespace ct {
namespace core {
namespace internal {


//! Trait defining basic math functions for CppAD CodeGen Auto-Diff types
class CppADCodegenTrait
{
public:
    typedef CppAD::AD<CppAD::cg::CG<double>> Scalar;

    inline static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar tan(const Scalar& x) { return CppAD::tan(x); }
    inline static Scalar sinh(const Scalar& x) { return CppAD::sinh(x); }
    inline static Scalar cosh(const Scalar& x) { return CppAD::cosh(x); }
    inline static Scalar tanh(const Scalar& x) { return CppAD::tanh(x); }
    inline static Scalar exp(const Scalar& x) { return CppAD::exp(x); }
    inline static Scalar fabs(const Scalar& x) { return CppAD::fabs(x); }
    inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
    //! Solves a linear system of equations using an LU factorization
    template <int Dims>
    inline static Eigen::Matrix<Scalar, Dims, 1> solve(const Eigen::Matrix<Scalar, Dims, Dims>& A,
        const Eigen::Matrix<Scalar, Dims, 1>& b)
    {
        Eigen::Matrix<Scalar, Dims, Dims> LU;
        Crout(A, LU);

        Eigen::Matrix<Scalar, Dims, 1> out;
        solveCrout(LU, b, out);

        return out;
    }


private:
    // Custom LU factorization
    template <typename T, int rowAndCol>  //only square matrices
    void static Crout(const Eigen::Matrix<T, rowAndCol, rowAndCol>& S, Eigen::Matrix<T, rowAndCol, rowAndCol>& D)
    {
        for (int k = 0; k < S.rows(); ++k)
        {
            for (int i = k; i < S.rows(); ++i)
            {
                T sum(0.);
                for (int p = 0; p < k; ++p)
                    sum += D(i, p) * D(p, k);
                D(i, k) = S(i, k) - sum;  // not dividing by diagonals
            }
            for (int j = k + 1; j < S.rows(); ++j)
            {
                T sum(0.);
                for (int p = 0; p < k; ++p)
                    sum += D(k, p) * D(p, j);
                D(k, j) = (S(k, j) - sum) / D(k, k);
            }
        }
    }

    // Custom LU solver
    template <typename T, int rowAndCol>
    void static solveCrout(const Eigen::Matrix<T, rowAndCol, rowAndCol>& LU,
        const Eigen::Matrix<T, rowAndCol, 1>& b,
        Eigen::Matrix<T, rowAndCol, 1>& x)
    {
        const int d = rowAndCol;
        T y[d];
        for (int i = 0; i < d; ++i)
        {
            T sum(0.0);
            for (int k = 0; k < i; ++k)
                sum += LU(i, k) * y[k];
            y[i] = (b(i) - sum) / LU(i, i);
        }
        for (int i = d - 1; i >= 0; --i)
        {
            T sum(0.);
            for (int k = i + 1; k < d; ++k)
                sum += LU(i, k) * x(k);
            x(i) = (y[i] - sum);  // not dividing by diagonals
        }
    }
};
}  // namespace internal
}  // namespace core
}  // namespace ct
