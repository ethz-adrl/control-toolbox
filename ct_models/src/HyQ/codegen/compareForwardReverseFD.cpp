/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#include <ct/rbd/rbd.h>

#include <ct/models/HyQ/HyQ.h>
#include <ct/models/HyQ/codegen/HyQWithContactModelLinearizedForward.h>
#include <ct/models/HyQ/codegen/HyQWithContactModelLinearizedReverse.h>
#include <ct/models/HyQ/codegen/HyQBareModelLinearizedForward.h>
#include <ct/models/HyQ/codegen/HyQBareModelLinearizedReverse.h>

using namespace ct::models::HyQ;

void timing(bool useContactModel)
{
    std::cout << std::boolalpha << "Using contact model: " << useContactModel << std::endl;

    typedef ct::rbd::FloatingBaseFDSystem<ct::rbd::HyQ::Dynamics, false> HyQSystem;
    std::shared_ptr<HyQSystem> hyqSys = std::shared_ptr<HyQSystem>(new HyQSystem);

    typedef ct::rbd::EEContactModel<typename HyQSystem::Kinematics> ContactModel;
    std::shared_ptr<ContactModel> contactModel = std::shared_ptr<ContactModel>(new ContactModel(5000.0, 1000.0, 100.0,
        100.0, -0.02, ContactModel::VELOCITY_SMOOTHING::SIGMOID, hyqSys->dynamics().kinematicsPtr()));
    if (useContactModel)
        hyqSys->setContactModel(contactModel);

    ct::rbd::RbdLinearizer<HyQSystem> linearizer(hyqSys);
    ct::core::SystemLinearizer<HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM> sysLinearizer(hyqSys, false);

    typedef std::shared_ptr<ct::core::LinearSystem<HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM>> LinModelPtr;
    LinModelPtr linModelForward;
    LinModelPtr linModelReverse;

    if (useContactModel)
    {
        linModelForward = LinModelPtr(new HyQWithContactModelLinearizedForward);
        linModelReverse = LinModelPtr(new HyQWithContactModelLinearizedReverse);
    }
    else
    {
        linModelForward = LinModelPtr(new HyQBareModelLinearizedForward);
        linModelReverse = LinModelPtr(new HyQBareModelLinearizedReverse);
    }

    static const size_t nTests = 10000;

    typedef typename HyQSystem::StateVector X;
    typedef typename HyQSystem::ControlVector U;

    typedef Eigen::Matrix<double, HyQSystem::STATE_DIM, HyQSystem::STATE_DIM> JacA;
    typedef Eigen::Matrix<double, HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM> JacB;


    std::vector<X, Eigen::aligned_allocator<X>> x(nTests);
    std::vector<U, Eigen::aligned_allocator<U>> u(nTests);

    std::vector<JacA, Eigen::aligned_allocator<JacA>> forwardA(nTests);
    std::vector<JacA, Eigen::aligned_allocator<JacA>> reverseA(nTests);
    std::vector<JacA, Eigen::aligned_allocator<JacA>> rbdA(nTests);
    std::vector<JacA, Eigen::aligned_allocator<JacA>> numDiffA(nTests);

    std::vector<JacB, Eigen::aligned_allocator<JacB>> forwardB(nTests);
    std::vector<JacB, Eigen::aligned_allocator<JacB>> reverseB(nTests);
    std::vector<JacB, Eigen::aligned_allocator<JacB>> rbdB(nTests);
    std::vector<JacB, Eigen::aligned_allocator<JacB>> numDiffB(nTests);

    std::cout << "input dim: " << HyQSystem::STATE_DIM + HyQSystem::CONTROL_DIM
              << ", output dim A: " << HyQSystem::STATE_DIM << ", output dim B: " << HyQSystem::CONTROL_DIM
              << std::endl;

    std::cout << "running " << nTests << " tests" << std::endl;
    for (size_t i = 0; i < nTests; i++)
    {
        x[i].setRandom();
        u[i].setRandom();
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        forwardA[i] = linModelForward->getDerivativeState(x[i], u[i]);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto diff = end - start;
    double msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "forwardA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        reverseA[i] = linModelReverse->getDerivativeState(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "reverseA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        rbdA[i] = linearizer.getDerivativeState(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "rbdA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        numDiffA[i] = sysLinearizer.getDerivativeState(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "numDiffA: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        forwardB[i] = linModelForward->getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "forwardB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        reverseB[i] = linModelReverse->getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "reverseB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        rbdB[i] = linearizer.getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "rbdB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        numDiffB[i] = sysLinearizer.getDerivativeControl(x[i], u[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "numDiffB: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;


    bool failed = false;
    for (size_t i = 0; i < nTests; i++)
    {
        if (!forwardA[i].isApprox(rbdA[i], 1e-5))
        {
            std::cout << "Forward A and RbdLinearizer A not similar" << std::endl;
            std::cout << "forward A: " << std::endl << forwardA[i] << std::endl;
            std::cout << "rbd A: " << std::endl << rbdA[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (!reverseA[i].isApprox(rbdA[i], 1e-5))
        {
            std::cout << "Reverse A and RbdLinearizer A not similar" << std::endl;
            std::cout << "reverse A: " << std::endl << reverseA[i] << std::endl;
            std::cout << "rbd A: " << std::endl << rbdA[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (!forwardA[i].isApprox(reverseA[i], 1e-12))
        {
            std::cout << "Forward A and reverse A not similar" << std::endl;
            std::cout << "forward A: " << std::endl << forwardA[i] << std::endl;
            std::cout << "reverse A: " << std::endl << reverseA[i] << std::endl << std::endl << std::endl;
            failed = true;
        }
        if (failed)
        {
            std::cout << "test failed, aborting" << std::endl;
            break;
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


int main(int argc, char* argv[])
{
    timing(true);
    std::cout << std::endl;
    timing(false);
    return 0;
}
