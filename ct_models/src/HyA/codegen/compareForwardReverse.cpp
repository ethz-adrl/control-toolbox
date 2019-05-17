/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <ct/models/HyA/HyA.h>
#include <ct/models/HyA/codegen/HyALinearizedForward.h>
#include <ct/models/HyA/codegen/HyALinearizedReverse.h>

#include <ct/models/HyA/codegen/HyAInverseDynJacForward.h>
#include <ct/models/HyA/codegen/HyAInverseDynJacReverse.h>

using namespace ct::models::HyA;

void timingForwardDynamics()
{
    HyALinearizedForward linModelForward;
    HyALinearizedReverse linModelReverse;

    typedef ct::rbd::FixBaseFDSystem<ct::rbd::HyA::Dynamics> HyASystem;
    std::shared_ptr<HyASystem> hyaSys = std::shared_ptr<HyASystem>(new HyASystem);

    ct::rbd::RbdLinearizer<HyASystem> linearizer(hyaSys);
    ct::core::SystemLinearizer<HyASystem::STATE_DIM, HyASystem::CONTROL_DIM> sysLinearizer(hyaSys, false);


    static const size_t nTests = 10000;

    typedef typename HyASystem::StateVector X;
    typedef typename HyASystem::ControlVector U;

    typedef Eigen::Matrix<double, HyASystem::STATE_DIM, HyASystem::STATE_DIM> JacA;
    typedef Eigen::Matrix<double, HyASystem::STATE_DIM, HyASystem::CONTROL_DIM> JacB;


    std::vector<X> x(nTests);
    std::vector<U> u(nTests);

    std::vector<JacA, Eigen::aligned_allocator<JacA>> forwardA(nTests);
    std::vector<JacA, Eigen::aligned_allocator<JacA>> reverseA(nTests);
    std::vector<JacA, Eigen::aligned_allocator<JacA>> rbdA(nTests);
    std::vector<JacA, Eigen::aligned_allocator<JacA>> numDiffA(nTests);

    std::vector<JacB, Eigen::aligned_allocator<JacB>> forwardB(nTests);
    std::vector<JacB, Eigen::aligned_allocator<JacB>> reverseB(nTests);
    std::vector<JacB, Eigen::aligned_allocator<JacB>> rbdB(nTests);
    std::vector<JacB, Eigen::aligned_allocator<JacB>> numDiffB(nTests);


    std::cout << "running " << nTests << " tests" << std::endl;
    for (size_t i = 0; i < nTests; i++)
    {
        x[i].setRandom();
        u[i].setRandom();
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        forwardA[i] = linModelForward.getDerivativeState(x[i], u[i]);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto diff = end - start;
    size_t msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "forwardA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        reverseA[i] = linModelReverse.getDerivativeState(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "reverseA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        rbdA[i] = linearizer.getDerivativeState(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "rbdA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        numDiffA[i] = sysLinearizer.getDerivativeState(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "numDiffA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        forwardB[i] = linModelForward.getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "forwardB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        reverseB[i] = linModelForward.getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "reverseB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        rbdB[i] = linearizer.getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "rbdB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        numDiffB[i] = sysLinearizer.getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "numDiffB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    bool failed = false;
    for (size_t i = 0; i < nTests; i++)
    {
        if (!forwardA[i].isApprox(reverseA[i], 1e-12))
        {
            std::cout << "Forward A and reverse A not similar" << std::endl;
            std::cout << "forward A: " << std::endl << forwardA[i] << std::endl;
            std::cout << "reverse A: " << std::endl << reverseA[i] << std::endl;
            failed = true;
        }

        if (!forwardA[i].isApprox(rbdA[i], 1e-5))
        {
            std::cout << "Forward A and RbdLinearizer A not similar" << std::endl;
            std::cout << "forward A: " << std::endl << forwardA[i] << std::endl;
            std::cout << "reverse A: " << std::endl << rbdA[i] << std::endl;
            failed = true;
        }

        if (!reverseA[i].isApprox(rbdA[i], 1e-5))
        {
            std::cout << "Forward A and RbdLinearizer A not similar" << std::endl;
            std::cout << "reverse A: " << std::endl << reverseA[i] << std::endl;
            std::cout << "rbd A: " << std::endl << rbdA[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
    }

    failed = false;
    for (size_t i = 0; i < nTests; i++)
    {
        if (!forwardB[i].isApprox(rbdB[i], 1e-5))
        {
            std::cout << "Forward B and RbdLinearizer B not similar" << std::endl;
            std::cout << "forward B: " << std::endl << forwardB[i] << std::endl;
            std::cout << "rbd B: " << std::endl << rbdB[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (!reverseB[i].isApprox(rbdB[i], 1e-5))
        {
            std::cout << "Forward B and RbdLinearizer B not similar" << std::endl;
            std::cout << "reverse B: " << std::endl << reverseB[i] << std::endl;
            std::cout << "rbd B: " << std::endl << rbdB[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (!forwardB[i].isApprox(reverseB[i], 1e-12))
        {
            std::cout << "Forward B and reverse B not similar" << std::endl;
            std::cout << "forward B: " << std::endl << forwardB[i] << std::endl;
            std::cout << "reverse B: " << std::endl << reverseB[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (failed)
        {
            std::cout << "test failed, aborting" << std::endl;
            break;
        }
    }
}

void timingInverseDynamics()
{
    HyAInverseDynJacForward ivLinearForward;
    HyAInverseDynJacReverse ivLinearReverse;

    typedef Eigen::Matrix<double, 6, 12> JacIv;
    static const size_t nTests = 10000;

    std::cout << "running " << nTests << " tests" << std::endl;

    std::vector<JacIv, Eigen::aligned_allocator<JacIv>> forwardJacIv(nTests);
    std::vector<JacIv, Eigen::aligned_allocator<JacIv>> reverseJacIv(nTests);

    typedef typename Eigen::Matrix<double, 12, 1> X;
    std::vector<X> x(nTests);

    for (size_t i = 0; i < nTests; i++)
    {
        x[i].setRandom();
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        forwardJacIv[i] = ivLinearForward.jacobian(x[i]);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto diff = end - start;
    size_t msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "forwardJacIv: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        reverseJacIv[i] = ivLinearReverse.jacobian(x[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::milli>(diff).count();
    std::cout << "reverseJacIv: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;


    bool failed = false;
    for (size_t i = 0; i < nTests; i++)
    {
        if (!forwardJacIv[i].isApprox(reverseJacIv[i], 1e-12))
        {
            std::cout << "ForwardJacIv and reverseJacIv not similar" << std::endl;
            std::cout << "ForwardJacIv A: " << std::endl << forwardJacIv[i] << std::endl;
            std::cout << "reverseJacIv A: " << std::endl << reverseJacIv[i] << std::endl;
            failed = true;
        }
    }
}

int main(int argc, char* argv[])
{
    timingForwardDynamics();
    timingInverseDynamics();
    return 0;
}
