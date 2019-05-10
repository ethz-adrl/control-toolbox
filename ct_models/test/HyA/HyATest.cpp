/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <memory>
#include <array>

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <ct/core/core.h>
#include <ct/rbd/rbd.h>

#include "ct/models/HyA/HyA.h"

using namespace ct;
using namespace ct::rbd;

#define Debug

TEST(HyaLinearizerTest, NumDiffComparison)
{
    typedef FixBaseFDSystem<HyA::Dynamics> HyASystem;

    const size_t STATE_DIM = HyASystem::STATE_DIM;
    const size_t CONTROL_DIM = HyASystem::CONTROL_DIM;

    std::shared_ptr<HyASystem> hyaSystem(new HyASystem);
    std::shared_ptr<HyASystem> hyaSystem2(new HyASystem);

    RbdLinearizer<HyASystem> rbdLinearizer(hyaSystem, true);
    core::SystemLinearizer<STATE_DIM, CONTROL_DIM> systemLinearizer(hyaSystem2, true);

    core::StateVector<STATE_DIM> x;
    x.setZero();
    core::ControlVector<CONTROL_DIM> u;
    u.setZero();

    auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 1.0);
    auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 1.0);

    auto A_system = systemLinearizer.getDerivativeState(x, u, 1.0);
    auto B_system = systemLinearizer.getDerivativeControl(x, u, 1.0);

    ASSERT_LT((A_rbd - A_system).array().abs().maxCoeff(), 1e-5);
    ASSERT_LT((B_rbd - B_system).array().abs().maxCoeff(), 1e-4);

    size_t nTests = 100;
    for (size_t i = 0; i < nTests; i++)
    {
        x.setRandom();
        u.setRandom();

        auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 0.0);
        auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 0.0);

        auto A_system = systemLinearizer.getDerivativeState(x, u, 0.0);
        auto B_system = systemLinearizer.getDerivativeControl(x, u, 0.0);

        ASSERT_LT((A_rbd - A_system).array().abs().maxCoeff(), 1e-5);

        ASSERT_LT((B_rbd - B_system).array().abs().maxCoeff(), 1e-4);
    }
}

TEST(CodegenLinearizerTest, NumDiffComparison)
{
    typedef FixBaseFDSystem<HyA::Dynamics> HyASystem;

    const size_t STATE_DIM = HyASystem::STATE_DIM;
    const size_t CONTROL_DIM = HyASystem::CONTROL_DIM;

    std::shared_ptr<HyASystem> hyaSystem(new HyASystem);

    RbdLinearizer<HyASystem> rbdLinearizer(hyaSystem, true);

    ct::models::HyA::HyALinearizedForward hyaLinear;

    core::StateVector<STATE_DIM> x;
    core::ControlVector<CONTROL_DIM> u;

    size_t nTests = 100;
    for (size_t i = 0; i < nTests; i++)
    {
        x.setRandom();
        u.setRandom();

        auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 0.0);
        auto A_gen = hyaLinear.getDerivativeState(x, u, 0.0);

        auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 0.0);
        auto B_gen = hyaLinear.getDerivativeControl(x, u, 0.0);

        ASSERT_LT((A_rbd - A_gen).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_rbd - B_gen).array().abs().maxCoeff(), 1e-4);
    }
}

TEST(IntegratorTest, IntegratorTestHya)
{
    typedef FixBaseFDSystem<HyA::Dynamics> HyASystem;

    const size_t STATE_DIM = HyASystem::STATE_DIM;

    std::shared_ptr<HyASystem> hyaSystem(new HyASystem);

    core::Integrator<STATE_DIM> integratorEulerOdeint(hyaSystem, core::EULER);
    core::Integrator<STATE_DIM> integratorRk4Odeint(hyaSystem, core::RK4);

    core::Integrator<STATE_DIM> integratorEulerCT(hyaSystem, core::EULERCT);
    core::Integrator<STATE_DIM> integratorRK4CT(hyaSystem, core::RK4CT);

    double dt = 0.001;
    double startTime = 0.0;
    size_t numSteps = 10;

    size_t nTests = 100;
    std::vector<core::StateVector<STATE_DIM>, Eigen::aligned_allocator<core::StateVector<STATE_DIM>>> xEulerOdeint(
        nTests),
        xEulerCt(nTests), xRk4Odeint(nTests), xRk4CT(nTests);

    for (size_t i = 0; i < nTests; ++i)
    {
        xEulerOdeint[i].setRandom();
        xEulerCt[i] = xEulerOdeint[i];
        xRk4Odeint[i].setRandom();
        xRk4CT[i] = xRk4Odeint[i];
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        integratorEulerOdeint.integrate_n_steps(xEulerOdeint[i], startTime, numSteps, dt);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto diff = end - start;
    double msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "integratorEulerOdeint: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms"
              << std::endl;


    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        integratorEulerCT.integrate_n_steps(xEulerCt[i], startTime, numSteps, dt);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "integratorEulerCT: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        integratorRk4Odeint.integrate_n_steps(xRk4Odeint[i], startTime, numSteps, dt);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "integratorRk4Odeint: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms"
              << std::endl;


    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < nTests; i++)
    {
        integratorRK4CT.integrate_n_steps(xRk4CT[i], startTime, numSteps, dt);
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    msTotal = std::chrono::duration<double, std::micro>(diff).count() / 1000.0;
    std::cout << "integratorRK4CT: " << msTotal << " ms. Average: " << msTotal / double(nTests) << " ms" << std::endl;


    for (size_t i = 0; i < nTests; ++i)
    {
        ASSERT_LT((xRk4CT[i] - xRk4Odeint[i]).array().abs().maxCoeff(), 1e-12);
        ASSERT_LT((xEulerCt[i] - xEulerOdeint[i]).array().abs().maxCoeff(), 1e-12);
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
