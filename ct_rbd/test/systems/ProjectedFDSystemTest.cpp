/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef TEST_SYSTEMS_PROJECTEDFDSYSTEMTEST_CPP_
#define TEST_SYSTEMS_PROJECTEDFDSYSTEMTEST_CPP_

#include <ct/rbd/rbd.h>
#include <memory>
#include <array>

#include <gtest/gtest.h>

#include "../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct;
using namespace ct::rbd;

TEST(ProjectedFDSystemTest, projected_forward_dynamics_test)
{
    const size_t STATE_DIM_QUAT = ProjectedFDSystem<TestHyQ::Dynamics, true>::STATE_DIM;
    const size_t STATE_DIM_EULER = ProjectedFDSystem<TestHyQ::Dynamics, false>::STATE_DIM;

    std::shared_ptr<ProjectedFDSystem<TestHyQ::Dynamics, true>> dynamicsQuat(
        new ProjectedFDSystem<TestHyQ::Dynamics, true>);
    std::shared_ptr<ProjectedFDSystem<TestHyQ::Dynamics, false>> dynamicsEuler(
        new ProjectedFDSystem<TestHyQ::Dynamics, false>);

    core::Integrator<STATE_DIM_QUAT> integratorQuat(dynamicsQuat, ct::core::RK4);
    core::Integrator<STATE_DIM_EULER> integratorEuler(dynamicsEuler, ct::core::RK4);

    TestHyQ::Kinematics kinematics;

    const size_t nTests = 100;
    const size_t nIntegrationSteps = 50;

    RBDState<12> state;

    for (size_t i = 0; i < nTests; i++)
    {
        state.setRandom();

        TestHyQ::Dynamics::EE_in_contact_t contactFlags;
        Eigen::Matrix<bool, 4, 1> eeContactConfig;
        eeContactConfig.setRandom();

        for (size_t j = 0; j < 4; j++)
        {
            contactFlags[j] = eeContactConfig[j];
        }

        dynamicsQuat->setEEInContact(contactFlags);
        dynamicsEuler->setEEInContact(contactFlags);

        core::StateVector<STATE_DIM_QUAT> stateQuat = state.toStateVectorQuaternion();
        core::StateVector<STATE_DIM_EULER> stateEuler = state.toStateVectorEulerXyz();

        RBDState<12> finalQuat, finalEuler;

        for (size_t j = 0; j < nIntegrationSteps; j++)
        {
            integratorQuat.integrate_n_steps(stateQuat, 0, 1, 0.001);

            integratorEuler.integrate_n_steps(stateEuler, 0, 1, 0.001);

            finalQuat.fromStateVectorQuaternion(stateQuat);
            finalEuler.fromStateVectorEulerXyz(stateEuler);

            //			std::cout << "test "<<i<<", integration: "<<j<<std::endl;
            //			std::cout << "diff: " << finalQuat.toStateVectorEulerXyz().transpose() - finalEuler.toStateVectorEulerXyz().transpose()<<std::endl;

            // check if both states are the same
            ASSERT_TRUE(finalQuat.isApprox(finalEuler, 1e-4));

            // check that EE in contact do not move
            for (size_t k = 0; k < 4; k++)
            {
                if (contactFlags[k])
                {
                    kinematics.getEEVelocityInWorld(k, finalQuat)
                        .toImplementation()
                        .isApprox(Eigen::Vector3d::Zero(), 1e-6);
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


#endif /* TEST_SYSTEMS_PROJECTEDFDSYSTEMTEST_CPP_ */
