/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <ct/models/HyQ/HyQ.h>
#include <ct/models/HyQ/codegen/HyQInverseDynJacForward.h>
#include <ct/models/HyQ/codegen/HyQInverseDynJacReverse.h>

#include "helperFunctions.h"

using namespace ct::models::HyQ;

void timing()
{
    std::cout << "Timing inverse dynamics " << std::endl;
    static const size_t nTests = 10000;

    const size_t IN_DIM = state_dim + 18;
    const size_t OUT_DIM = control_dim + 6;
    typedef Eigen::Matrix<double, IN_DIM, 1> X;
    typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> Jac;

    std::vector<X, Eigen::aligned_allocator<X>> x(nTests);

    std::vector<Jac, Eigen::aligned_allocator<Jac>> forward(nTests);
    std::vector<Jac, Eigen::aligned_allocator<Jac>> reverse(nTests);
    std::vector<Jac, Eigen::aligned_allocator<Jac>> numDiff(nTests);

    HyQInverseDynJacForward forwardJac;
    HyQInverseDynJacReverse reverseJac;
    ct::core::DerivativesNumDiff<IN_DIM, OUT_DIM>::Function idFun = ct::models::HyQ::hyqInverseDynamics<double>;
    ct::core::DerivativesNumDiff<IN_DIM, OUT_DIM> numDiffJac(idFun);


    std::cout << "input dim: " << IN_DIM << ", output dim: " << OUT_DIM << std::endl;

    std::cout << "running " << nTests << " tests" << std::endl;
    for (size_t i = 0; i < nTests; i++)
    {
        x[i].setRandom();
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        forward[i] = forwardJac.jacobian(x[i]);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto diff = end - start;
    size_t msTotal = std::chrono::duration<double, std::micro>(diff).count();
    std::cout << "forwardA: " << msTotal / 1000.0 << " ms. Average: " << msTotal / double(nTests) / 1000.0 << " ms"
              << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        reverse[i] = reverseJac.jacobian(x[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count();
    std::cout << "reverseA: " << msTotal / 1000.0 << " ms. Average: " << msTotal / double(nTests) / 1000.0 << " ms"
              << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        numDiff[i] = numDiffJac.jacobian(x[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count();
    std::cout << "numDiffA: " << msTotal / 1000.0 << " ms. Average: " << msTotal / double(nTests) / 1000.0 << " ms"
              << std::endl;


    bool failed = false;
    for (size_t i = 0; i < nTests; i++)
    {
        if (!forward[i].isApprox(numDiff[i], 1e-5))
        {
            std::cout << "Forward and NumDiff not similar" << std::endl;
            std::cout << "forward: " << std::endl << forward[i] << std::endl;
            std::cout << "numDiff: " << std::endl << numDiff[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (!forward[i].isApprox(reverse[i], 1e-12))
        {
            std::cout << "Forward and reverse not similar" << std::endl;
            std::cout << "forward: " << std::endl << forward[i] << std::endl;
            std::cout << "reverse: " << std::endl << reverse[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (failed)
        {
            std::cout << "test failed, aborting" << std::endl;
            break;
        }
    }
}


int main(int argc, char* argv[])
{
    timing();
    return 0;
}
