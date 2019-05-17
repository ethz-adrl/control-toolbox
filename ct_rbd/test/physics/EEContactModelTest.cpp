/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <memory>
#include <gtest/gtest.h>

#include <ct/rbd/physics/EEContactModel.h>

#include "../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;


TEST(EEContactModelTest, basicTest)
{
    typedef TestHyQ::Kinematics HyqKinematics;
    typedef typename EEContactModel<HyqKinematics>::EEForcesLinear EEForcesLinear;

    EEContactModel<HyqKinematics> eeContactModel;

    RBDState<HyqKinematics::NJOINTS> state;
    state.setRandom();

    EEForcesLinear forces;

    try
    {
        forces = eeContactModel.computeContactForces(state);
    } catch (const std::runtime_error& e)
    {
        std::cout << "error thrown: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }

    for (size_t i = 0; i < forces.size(); i++)
    {
        std::cout << "Force at EE-ID " << i << ": " << forces[i].transpose();
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
