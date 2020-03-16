/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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

TEST(MatrixInversionTest, CustomInversionTests)
{
    constexpr int NTESTS = 1000;
    MatrixNd A;
    VectorNd b, x_ct, x_eigen;
    Eigen::MatrixXd A_dynamic(n, n);
    Eigen::VectorXd b_dynamic(n), x_ct_dynamic(n);


    for (int i = 0; i < NTESTS; i++)
    {
        b.setRandom();
        A = getPositiveDefiniteMatrix();
        b_dynamic = b;
        A_dynamic = A;

        // Test LDLT
        x_ct = LDLTsolve<double>(A, b);
        x_ct_dynamic = LDLTsolve<double>(A_dynamic, b_dynamic);
        x_eigen = A.ldlt().solve(b);

        ASSERT_TRUE(x_ct.isApprox(x_eigen, 1e-9));
        ASSERT_TRUE(x_ct_dynamic.isApprox(x_eigen, 1e-9));

        // Test LU
        x_ct = LUsolve<double>(A, b);
        x_ct_dynamic = LUsolve<double>(A_dynamic, b_dynamic);
        x_eigen = A.lu().solve(b);

        ASSERT_TRUE(x_ct.isApprox(x_eigen, 1e-9));
        ASSERT_TRUE(x_ct_dynamic.isApprox(x_eigen, 1e-9));
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
