/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>

#include <ct/rbd/rbd.h>
#include <memory>
#include <array>

#include <iostream>

#include <gtest/gtest.h>

#include <ct/models/QuadrotorWithLoad/QuadrotorWithLoad.h>
#include <ct/models/QuadrotorWithLoad/QuadrotorWithLoadFDSystem.h>

using namespace ct;
using namespace ct::rbd;


TEST(QuadrotorWithLoadTestTest, NumDiffComparison)
{
    typedef QuadrotorWithLoadFDSystem<quadrotor::Dynamics> QuadrotorSystem;

    const size_t STATE_DIM = QuadrotorSystem::STATE_DIM;
    const size_t CONTROL_DIM = QuadrotorSystem::CONTROL_DIM;

    std::shared_ptr<QuadrotorSystem> quadrotorSystem(new QuadrotorSystem);

    core::SystemLinearizer<STATE_DIM, CONTROL_DIM> systemLinearizer(quadrotorSystem);

    core::StateVector<STATE_DIM> x;
    x.setZero();
    core::ControlVector<CONTROL_DIM> u;
    u.setZero();

    //	auto A_system = systemLinearizer.getDerivativeState(x, u, 1.0);
    //	auto B_system = systemLinearizer.getDerivativeControl(x, u, 1.0);
    //
    //
    //	size_t nTests = 1000;
    //	for (size_t i=0; i<nTests; i++)
    //	{
    //		x.setRandom();
    //		u.setRandom();
    //
    //		auto A_system = systemLinearizer.getDerivativeState(x, u, 0.0);
    //		auto B_system = systemLinearizer.getDerivativeControl(x, u, 0.0);
    //	}

    /* todo insert meaningful unit test here */
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
