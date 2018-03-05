/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

// Bring in gtest
#include <gtest/gtest.h>

#include <ct/core/core.h>

using namespace ct::core;

// Set the test size globally
constexpr size_t n = 20;
using MatrixNd = Eigen::Matrix<double, n, n>;
using VectorNd = Eigen::Matrix<double, n, 1>;

MatrixNd getRandomSymetricMatrix()
{
    MatrixNd A_;
    A_.setRandom();
    return A_ * A_.transpose();
}

Eigen::Matrix<double, n, n> getPositiveDefiniteMatrix()
{
    MatrixNd A_sym_ = getRandomSymetricMatrix();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
    svd_.compute(A_sym_, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd sing_values(svd_.matrixV().cols(), 1);  // size of E has same size as columns of V
    // Set all singular values to s = |s| + tol => s>=0
    sing_values = svd_.singularValues().array().abs();
    sing_values.array() += 1e-5;
    return svd_.matrixV() * sing_values.asDiagonal() * svd_.matrixU().transpose();
}

TEST(MatrixInversionTest, ADHelperFunctionsCustomInversionTests)
{
    constexpr int NTESTS = 1000;
    MatrixNd A;
    VectorNd b, x_ct, x_ct_dynamic, x_eigen;

    // Test LDLT
    for (int i = 0; i < NTESTS; i++)
    {
        b.setRandom();
        A = getPositiveDefiniteMatrix();

        x_ct = ADHelperFunctions::LDLTsolve<double, n, 1>(A, b);
        x_ct_dynamic = ADHelperFunctions::LDLTsolve<double>(A, b);
        x_eigen = A.ldlt().solve(b);

        ASSERT_TRUE(x_ct.isApprox(x_eigen, 1e-9));
        ASSERT_TRUE(x_ct_dynamic.isApprox(x_eigen, 1e-9));
    }

    // Test LU
    for (int i = 0; i < NTESTS; i++)
    {
        b.setRandom();
        A = getPositiveDefiniteMatrix();

        x_ct = ADHelperFunctions::LUsolve<double, n, 1>(A, b);
        x_eigen = A.ldlt().solve(b);

        ASSERT_TRUE(x_ct.isApprox(x_eigen, 1e-9));
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
