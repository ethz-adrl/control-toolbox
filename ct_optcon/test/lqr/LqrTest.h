/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This file implements a LQR unit tests and a comparison against MATLAB
 * For more intuitive examples, visit the tutorial.
 * \example LqrTest.h
 */

#pragma once

#include <chrono>

#ifdef MATLAB
#include <matlabCppInterface/Engine.hpp>
#endif

// Bring in gtest
#include <gtest/gtest.h>


namespace ct {
namespace optcon {
namespace example {

TEST(LQRTest, DARETest)
{
    const size_t stateDim = 2;
    const size_t controlDim = 1;

    Eigen::Matrix<double, stateDim, stateDim> A;
    Eigen::Matrix<double, stateDim, controlDim> B;
    Eigen::Matrix<double, stateDim, stateDim> Q;
    Eigen::Matrix<double, controlDim, controlDim> R;
    Eigen::Matrix<double, controlDim, stateDim> K;

    A << 1, 1, 1, 0;
    B << 0, 1;
    Q << 1, 0, 0, 1;
    R << 1;

    ct::optcon::DARE<stateDim, controlDim> dare;
    Eigen::Matrix<double, stateDim, stateDim> P = dare.computeSteadyStateRiccatiMatrix(Q, R, A, B, K, true);
    Eigen::Matrix<double, stateDim, stateDim> P_test;
    P_test << 6.932484752255643, 4.332273119899151, 4.332273119899151, 4.55195134961773;
    ASSERT_LT((P - P_test).array().abs().maxCoeff(), 1e-12);
}


TEST(LQRTest, quadTest)
{
    //	std::cout << "QUADROTOR TEST"<<std::endl;
    //	std::cout << "==================================="<<std::endl;
    //	std::cout << "==================================="<<std::endl << std::endl << std::endl;

    const size_t stateDim = 12;
    const size_t controlDim = 4;

    Eigen::Matrix<double, stateDim, stateDim> A;
    Eigen::Matrix<double, stateDim, controlDim> B;
    Eigen::Matrix<double, stateDim, stateDim> Q;
    Eigen::Matrix<double, controlDim, controlDim> R;
    Eigen::Matrix<double, controlDim, stateDim> K;
    Eigen::Matrix<double, controlDim, stateDim> Kiterative;

    ct::optcon::LQR<stateDim, controlDim> lqr;

    Q << 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2000, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0.02;

    R << 100, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 100;


    A << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, -0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, -0, 0, 0, 0, 0, -0, -0, 0, 0, 0, 0, -0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0,
        0;


    B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0, 0, 0, 1.39665, 0, 0,
        0, 0, 142.857, -0, 0, 0, 0, 142.857, 0, 0, -0, 0, 83.3333;

    Kiterative.setZero();
    K.setZero();


    bool foundSolutionIterative = lqr.compute(Q, R, A, B, Kiterative, false, true);
    ASSERT_TRUE(foundSolutionIterative);

#ifdef CT_USE_LAPACK
    bool foundSolutionDirect = lqr.compute(Q, R, A, B, K, false);
    ASSERT_TRUE(foundSolutionDirect);
    ASSERT_LT((K - Kiterative).array().abs().maxCoeff(), 1e-4);
#endif

    int nTests = 100;
#ifdef CT_USE_LAPACK
    auto start1 = std::chrono::system_clock::now();
    for (int i = 0; i < nTests; i++)
    {
        lqr.compute(Q, R, A, B, K, false);
    }
    auto end1 = std::chrono::system_clock::now();
    auto elapsed1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    std::cout << "solved " << nTests << " lqr problems with state dimension " << stateDim << " in " << elapsed1.count()
              << " ms (average: " << elapsed1.count() / static_cast<double>(nTests) << " ms / lqr)" << std::endl;
#endif

    auto start2 = std::chrono::system_clock::now();
    for (int i = 0; i < nTests; i++)
    {
        lqr.compute(Q, R, A, B, Kiterative, false, true);
    }
    auto end2 = std::chrono::system_clock::now();
    auto elapsed2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    std::cout << "solved " << nTests << " lqr problems iteratively with state dimension " << stateDim << " in "
              << elapsed2.count() << " ms (average: " << elapsed2.count() / static_cast<double>(nTests) << " ms / lqr)"
              << std::endl;
}

#ifdef MATLAB
TEST(LQRTest, matlabTest)
{
    matlab::Engine engine(true);
    ASSERT_TRUE(engine.good());

    const size_t stateDim = 5;
    std::string stateDimString = std::to_string(stateDim);

    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd Qd;
    Eigen::MatrixXd Rd;
    Eigen::MatrixXd K_Matlab;

    Eigen::Matrix<double, stateDim, stateDim> A;
    Eigen::Matrix<double, stateDim, stateDim> B;
    Eigen::Matrix<double, stateDim, stateDim> Q;
    Eigen::Matrix<double, stateDim, stateDim> R;
    Eigen::Matrix<double, stateDim, stateDim> K_Cpp;
    Eigen::Matrix<double, stateDim, stateDim> K_Cpp_iteratively;

    ct::optcon::LQR<stateDim, stateDim> lqr;

    //	std::cout << "ARTIFICIAL TEST "<<std::endl;
    //	std::cout << "==================================="<<std::endl;
    //	std::cout << "==================================="<<std::endl << std::endl << std::endl;

    for (int i = 0; i < 10; i++)
    {
        std::cout << "Test " << std::to_string(i) << std::endl;
        std::cout << "===================================" << std::endl;

        std::cout << "1. Generating problem in Matlab" << std::endl;
        engine.executeCommand("A = magic(" + stateDimString + ");");
        engine.executeCommand("B = magic(" + stateDimString + ");");
        engine.executeCommand("Q = diag(100*rand(" + stateDimString + ",1));");
        engine.executeCommand("R = diag(100*rand(" + stateDimString + ",1));");
        engine.executeCommand("N = zeros(" + stateDimString + ");");

        std::cout << "2. Computing LQR in Matlab" << std::endl;
        std::cout << engine.executeCommand("[K,S,E] = lqr(A,B,Q,R,N);");

        std::cout << "3. Obtaining problem from Matlab" << std::endl;
        engine.get("A", Ad);
        A = Ad;
        engine.get("B", Bd);
        B = Bd;
        engine.get("Q", Qd);
        Q = Qd;
        engine.get("R", Rd);
        R = Rd;

        std::cout << "4. Obtaining LQR solution from Matlab" << std::endl;
        engine.get("K", K_Matlab);

        std::cout << "5. Computing LQR solution in C++" << std::endl;

        bool foundSolutionDirect = lqr.compute(Q, R, A, B, K_Cpp, false);
        ASSERT_TRUE(foundSolutionDirect);

        bool foundSolutionIterative = lqr.compute(Q, R, A, B, K_Cpp_iteratively, false, true);
        ASSERT_TRUE(foundSolutionIterative);


        std::cout << "7. Comparing both solutions" << std::endl;
        ASSERT_LT((K_Matlab - K_Cpp).array().abs().maxCoeff(), 1e-4);
        ASSERT_LT((K_Matlab - K_Cpp_iteratively).array().abs().maxCoeff(), 1e-4);

        std::cout << std::endl << std::endl << std::endl;
    }
}
#endif  //MATLAB

}  // namespace example
}  // namespace optcon
}  // namespace ct
