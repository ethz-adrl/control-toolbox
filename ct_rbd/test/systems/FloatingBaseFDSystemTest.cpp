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

#include <gtest/gtest.h>

#include "../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct;
using namespace ct::rbd;

TEST(FloatingBaseFDSystemTest, forward_dynamics_test)
{
    const size_t STATE_DIM_QUAT = FloatingBaseFDSystem<TestHyQ::Dynamics, true>::STATE_DIM;
    const size_t STATE_DIM_EULER = FloatingBaseFDSystem<TestHyQ::Dynamics, false>::STATE_DIM;

    std::shared_ptr<core::System<STATE_DIM_QUAT>> dynamicsQuat(new FloatingBaseFDSystem<TestHyQ::Dynamics, true>);
    std::shared_ptr<core::System<STATE_DIM_EULER>> dynamicsEuler(new FloatingBaseFDSystem<TestHyQ::Dynamics, false>);

    core::Integrator<STATE_DIM_QUAT> integratorQuat(dynamicsQuat, ct::core::RK4);
    core::Integrator<STATE_DIM_EULER> integratorEuler(dynamicsEuler, ct::core::RK4);

    RBDState<12> state;
    state.setRandom();

    core::StateVector<STATE_DIM_QUAT> stateQuat = state.toStateVectorQuaternion();
    core::StateVector<STATE_DIM_EULER> stateEuler = state.toStateVectorEulerXyz();

    integratorQuat.integrate_n_steps(stateQuat, 0, 1000, 0.001);

    integratorEuler.integrate_n_steps(stateEuler, 0, 1000, 0.001);

    RBDState<12> finalQuat, finalEuler;
    finalQuat.fromStateVectorQuaternion(stateQuat);
    finalEuler.fromStateVectorEulerXyz(stateEuler);

    std::cout << "norm error between quat/euler integration: " << std::endl
              << (finalQuat.toStateVectorEulerXyz() - finalEuler.toStateVectorEulerXyz()).norm() << std::endl;

    ASSERT_TRUE(finalQuat.isApprox(finalEuler, 1e-6));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#pragma GCC diagnostic pop