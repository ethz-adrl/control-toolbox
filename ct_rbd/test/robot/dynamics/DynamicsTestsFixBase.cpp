/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <memory>
#include <array>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <ct/rbd/rbd.h>

#include "../../models/testIrb4600/RobCoGenTestIrb4600.h"
#include "ct/rbd/robot/Dynamics.h"
#include "ct/rbd/robot/Kinematics.h"

using namespace ct::rbd;

TEST(DynamicsTestIrb4600, forward_dynamics_test)
{
    std::shared_ptr<TestIrb4600::Kinematics> kyn(new TestIrb4600::Kinematics);

    TestIrb4600::Dynamics testdynamics(kyn);

    using control_vector_t = typename TestIrb4600::Dynamics::control_vector_t;
    using ForceVector_t = typename TestIrb4600::Dynamics::ForceVector_t;
    using JointState_t = typename TestIrb4600::Dynamics::JointState_t;
    using JointAcceleration_t = typename TestIrb4600::Dynamics::JointAcceleration_t;
    using ExtLinkForces_t = typename TestIrb4600::Dynamics::ExtLinkForces_t;

    JointState_t irb_state;
    irb_state.setZero();

    control_vector_t torque_u = control_vector_t::Zero();
    ExtLinkForces_t ext_forces;
    ext_forces = ForceVector_t::Zero();

    JointAcceleration_t irb_xd;

    testdynamics.FixBaseForwardDynamics(irb_state, torque_u, ext_forces, irb_xd);

    JointAcceleration_t qdd;
    testdynamics.FixBaseID(irb_state, qdd, ext_forces, torque_u);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
