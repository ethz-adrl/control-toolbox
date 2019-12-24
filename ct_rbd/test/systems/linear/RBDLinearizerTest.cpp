/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <iostream>

#include <gtest/gtest.h>

#include <ct/rbd/systems/linear/RbdLinearizer.h>
#include "ct/rbd/systems/FixBaseFDSystem.h"
#include "ct/rbd/systems/FloatingBaseFDSystem.h"

#include "../../models/testIrb4600/RobCoGenTestIrb4600.h"
#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct;
using namespace ct::rbd;

TEST(RBDLinearizerTest, NumDiffComparisonFixedBase)
{
    typedef FixBaseFDSystem<TestIrb4600::Dynamics> IrbSystem;

    const size_t STATE_DIM = IrbSystem::STATE_DIM;
    const size_t CONTROL_DIM = IrbSystem::CONTROL_DIM;

    std::shared_ptr<IrbSystem> irbSystem(new IrbSystem);
    std::shared_ptr<IrbSystem> irbSystem2(new IrbSystem);

    RbdLinearizer<IrbSystem> rbdLinearizer(irbSystem, true);
    core::SystemLinearizer<STATE_DIM, CONTROL_DIM> systemLinearizer(irbSystem2, true);

    core::StateVector<STATE_DIM> x;
    core::ControlVector<CONTROL_DIM> u;

    size_t nTests = 500;
    for (size_t i = 0; i < nTests; i++)
    {
        x.setRandom();
        u.setRandom();

        auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 0.0);
        auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 0.0);

        auto A_system = systemLinearizer.getDerivativeState(x, u, 0.0);
        auto B_system = systemLinearizer.getDerivativeControl(x, u, 0.0);

        //		std::cout << "A_rbd" 	<< std::endl << A_rbd 			<< std::endl << std::endl;
        //		std::cout << "A_system" << std::endl << A_system 		<< std::endl << std::endl;
        //		std::cout << "Diff:" 	<< std::endl << A_rbd-A_system 	<< std::endl << std::endl;

        //		std::cout << "B_rbd" 	<< std::endl << B_rbd			<< std::endl << std::endl;
        //		std::cout << "B_system" << std::endl << B_system 		<< std::endl << std::endl;
        //		std::cout << "Diff:" 	<< std::endl << B_rbd-B_system 	<< std::endl << std::endl;

        ASSERT_LT((A_rbd - A_system).array().abs().maxCoeff(), 1e-5);

        ASSERT_LT((B_rbd - B_system).array().abs().maxCoeff(), 1e-4);
    }
}

TEST(RBDLinearizerTest, NumDiffComparisonFloatingBase)
{
    typedef FloatingBaseFDSystem<TestHyQ::Dynamics, false, false> HyQSystem;

    const size_t STATE_DIM = HyQSystem::STATE_DIM;
    const size_t CONTROL_DIM = HyQSystem::CONTROL_DIM;

    std::shared_ptr<HyQSystem> hyqSystem(new HyQSystem);
    std::shared_ptr<HyQSystem> hyqSystem2(new HyQSystem);

    RbdLinearizer<HyQSystem> rbdLinearizer(hyqSystem, true);
    core::SystemLinearizer<STATE_DIM, CONTROL_DIM> systemLinearizer(hyqSystem2, true);

    core::StateVector<STATE_DIM> x;
    core::ControlVector<CONTROL_DIM> u;

    size_t nTests = 500;
    for (size_t i = 0; i < nTests; i++)
    {
        x.setRandom();
        u.setRandom();

        auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 0.0);
        auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 0.0);

        auto A_system = systemLinearizer.getDerivativeState(x, u, 0.0);
        auto B_system = systemLinearizer.getDerivativeControl(x, u, 0.0);

        //		std::cout << "A_rbd" << std::endl << A_rbd << std::endl << std::endl;
        //		std::cout << "A_system" << std::endl << A_system << std::endl << std::endl;
        //
        //		std::cout << "Diff:" << std::endl << A_rbd-A_system << std::endl << std::endl;

        ASSERT_LT((A_rbd - A_system).array().abs().maxCoeff(), 1e-5);

        //		std::cout << "B_rbd" << std::endl << B_rbd << std::endl << std::endl;
        //		std::cout << "B_system" << std::endl << B_system << std::endl << std::endl;
        //		std::cout << "Diff:" << std::endl << B_rbd-B_system << std::endl << std::endl;

        ASSERT_LT((B_rbd - B_system).array().abs().maxCoeff(), 1e-4);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


#pragma GCC diagnostic pop