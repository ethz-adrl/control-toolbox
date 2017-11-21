/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>

#include <ct/core/core.h>
#include <ct/rbd/rbd.h>

#include <ct/rbd/systems/linear/RbdLinearizer.h>

#include <models/testIrb4600/RobCoGenTestIrb4600.h>


TEST(RbdControllerTests, RbdControllerTests)
{
    /*
	const size_t state_dim   = ct::rbd::FixBaseFDSystem<ct::rbd::TestIrb4600::Dynamics>::STATE_DIM;
	const size_t control_dim = ct::rbd::FixBaseFDSystem<ct::rbd::TestIrb4600::Dynamics>::CONTROL_DIM;

	std::shared_ptr<ct::rbd::FixBaseFDSystem<ct::rbd::TestIrb4600::Dynamics>> robot (new ct::rbd::FixBaseFDSystem<ct::rbd::TestIrb4600::Dynamics>);

	ct::core::SystemLinearizer<state_dim, control_dim> systemLinearizer(robot);

	todo: unit test ausbauen, gestoppt da irb system noch nicht korrekt.

*/
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
