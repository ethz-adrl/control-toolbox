/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace ADHelperFunctions {

// Custom LU factorization
template <typename SCALAR, int n>  //only square matrices
void lu(const Eigen::Matrix<SCALAR, n, n>& S, Eigen::Matrix<SCALAR, n, n>& D)
{
    for (int k = 0; k < n; ++k)
    {
        for (int i = k; i < n; ++i)
        {
            SCALAR sum(0.);
            for (int p = 0; p < k; ++p)
            {
                sum += D(i, p) * D(p, k);
            }
            D(i, k) = S(i, k) - sum;  // not dividing by diagonals
        }
        for (int j = k + 1; j < n; ++j)
        {
            SCALAR sum(0.);
            for (int p = 0; p < k; ++p)
            {
                sum += D(k, p) * D(p, j);
            }
            D(k, j) = (S(k, j) - sum) / D(k, k);
        }
    }
}

// Custom LU solver
template <typename SCALAR, int n, int m>
void solveLU(const Eigen::Matrix<SCALAR, n, n>& LU,
    const Eigen::Matrix<SCALAR, n, m>& b,
    Eigen::Matrix<SCALAR, n, m>& x)
{
    SCALAR y[n];
    for (int j = 0; j < m; ++j)
    {
        // Solve LU for jth-column on b
        for (int i = 0; i < n; ++i)
        {
            SCALAR sum(0.0);
            for (int k = 0; k < i; ++k)
            {
                sum += LU(i, k) * y[k];
            }
            y[i] = (b(i, j) - sum) / LU(i, i);
        }
        for (int i = n - 1; i >= 0; --i)
        {
            SCALAR sum(0.);
            for (int k = i + 1; k < n; ++k)
            {
                sum += LU(i, k) * x(k, j);
            }
            x(i, j) = (y[i] - sum);  // not dividing by diagonals
        }
    }
}

/***
 *  LU-based inverse solver
 *  This implementation does NOT use pivoting. Therefore we can only invert well-conditioned positive definite matrices
 *	having a positive semi-definite A is not tested
 */
template <typename SCALAR, int n, int m>  // solves X = A^(-1) * B, with A nxn and b nxm
Eigen::Matrix<SCALAR, n, m> LUsolve(const Eigen::Matrix<SCALAR, n, n>& A, const Eigen::Matrix<SCALAR, n, m>& B)
{
    Eigen::Matrix<SCALAR, n, n> LU;
    lu<SCALAR, n>(A, LU);

    Eigen::Matrix<SCALAR, n, m> X;
    solveLU<SCALAR, n, m>(LU, B, X);

    return X;
}


// Custom LDLT factorization (Algorithm from https://en.wikipedia.org/wiki/Cholesky_decomposition)
template <typename SCALAR, int n>  //only square matrices
void ldlt(const Eigen::Matrix<SCALAR, n, n>& A, Eigen::Matrix<SCALAR, n, n>& L, Eigen::Matrix<SCALAR, n, 1>& d)
{
    L.setIdentity();
    for (int j = 0; j < n; ++j)
    {
        SCALAR sum(0.);
        for (int k = 0; k < j; ++k)
        {
            sum += L(j, k) * L(j, k) * d(k);
        }
        d(j) = A(j, j) - sum;

        for (int i = j + 1; i < n; i++)
        {
            SCALAR sum(0.);
            for (int k = 0; k < j; ++k)
            {
                sum += L(i, k) * L(j, k) * d(k);
            }
            L(i, j) = (A(i, j) - sum) / d(j);
        }
    }
}
// Custom LDLT factorization (Algorithm from https://en.wikipedia.org/wiki/Cholesky_decomposition)
template <typename SCALAR>  //only square matrices
void ldlt_dynamic(const Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& A,
    Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& L,
    Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& d)
{
    const int n = A.rows();
    L.setIdentity();
    for (int j = 0; j < n; ++j)
    {
        SCALAR sum(0.);
        for (int k = 0; k < j; ++k)
        {
            sum += L(j, k) * L(j, k) * d(k);
        }
        d(j) = A(j, j) - sum;

        for (int i = j + 1; i < n; i++)
        {
            SCALAR sum(0.);
            for (int k = 0; k < j; ++k)
            {
                sum += L(i, k) * L(j, k) * d(k);
            }
            L(i, j) = (A(i, j) - sum) / d(j);
        }
    }
}

// Custom LDLT solver
template <typename SCALAR, int n, int m>  // solves X = A^(-1) * B, with A nxn and b nxm
void solveLDLT(const Eigen::Matrix<SCALAR, n, n>& L,
    const Eigen::Matrix<SCALAR, n, 1>& d,
    const Eigen::Matrix<SCALAR, n, m>& b,
    Eigen::Matrix<SCALAR, n, m>& x)
{
    SCALAR y[n];
    for (int j = 0; j < m; ++j)
    {
        // Solve LDLT for j-th column of b
        // 1. solve L*y = b
        for (int i = 0; i < n; ++i)
        {
            SCALAR sum(0.);
            for (int k = 0; k < i; k++)
            {
                sum += L(i, k) * y[k];
            }
            y[i] = (b(i, j) - sum);
        }
        // 2. solve D L^T  x = y
        for (int i = n - 1; i >= 0; --i)
        {
            SCALAR sum(0.);
            for (int k = i + 1; k < n; ++k)
            {
                sum += L(k, i) * x(k, j);
            }
            x(i, j) = (y[i] / d(i) - sum);
        }
    }
}

// Custom LDLT solver
template <typename SCALAR>  // solves X = A^(-1) * B, with A nxn and b nxm
void solveLDLT_dynamic(const Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& L,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& d,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& b,
    Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& x)
{
    const int n = L.rows();
    const int m = b.cols();
    Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> y(n);
    for (int j = 0; j < m; ++j)
    {
        // Solve LDLT for j-th column of b
        // 1. solve L*y = b
        for (int i = 0; i < n; ++i)
        {
            SCALAR sum(0.);
            for (int k = 0; k < i; k++)
            {
                sum += L(i, k) * y(k);
            }
            y(i) = (b(i, j) - sum);
        }
        // 2. solve D L^T  x = y
        for (int i = n - 1; i >= 0; --i)
        {
            SCALAR sum(0.);
            for (int k = i + 1; k < n; ++k)
            {
                sum += L(k, i) * x(k, j);
            }
            x(i, j) = (y(i) / d(i) - sum);
        }
    }
}

/***
 *  LDLT-based inverse solver
 *  Only use for symmetric(!) positive (semi)-definite matrices.
 *  This implementation does NOT use pivoting, but this algorithm has good stability properties for positive definite matrices in general.
 */
template <typename SCALAR, int n, int m>  // solves X = A^(-1) * B, with A nxn and b nxm
Eigen::Matrix<SCALAR, n, m> LDLTsolve(const Eigen::Matrix<SCALAR, n, n>& A, const Eigen::Matrix<SCALAR, n, m>& B)
{
    // Compute the decomposition
    Eigen::Matrix<SCALAR, n, n> L;
    Eigen::Matrix<SCALAR, n, 1> d;
    ldlt<SCALAR, n>(A, L, d);

    // Solve the LDLT * X = B system
    Eigen::Matrix<SCALAR, n, m> X;
    solveLDLT(L, d, B, X);
    return X;
}

/***
 *  LDLT-based inverse solver with dynamic sizing
 *  Only use for symmetric(!) positive (semi)-definite matrices.
 *  This implementation does NOT use pivoting, but this algorithm has good stability properties for positive definite matrices in general.
 */
template <typename SCALAR>  // solves X = A^(-1) * B, with A nxn and b nxm
Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> LDLTsolve_dynamic(
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>& B)
{
    // Compute the decomposition
    const int n = A.rows();
    const int m = B.cols();
    Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> L(n, n);
    Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> d(n);
    ldlt_dynamic(A, L, d);

    // Solve the LDLT * X = B system
    Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> X(n, m);
    solveLDLT_dynamic(L, d, B, X);
    return X;
}


}  // namespace ADHelperFunctions

class ADHelpers
{
public:
private:
};
}
}
