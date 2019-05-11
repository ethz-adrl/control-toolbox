/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <gtest/gtest.h>

#include "ct/rbd/robot/Dynamics.h"
#include "ct/rbd/robot/Kinematics.h"
#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;

TEST(DynamicsTestHyQ, forward_dynamics_test)
{
    typedef float valType;

    std::shared_ptr<TestHyQ::tpl::Kinematics<valType>> kyn(new TestHyQ::tpl::Kinematics<valType>);

    typedef TestHyQ::tpl::Dynamics<valType> Dyn;

    Dyn testdynamics(kyn);

    using control_vector_t = typename Dyn::control_vector_t;
    using ForceVector_t = typename Dyn::ForceVector_t;
    using RBDState_t = typename Dyn::RBDState_t;
    using RBDAcceleration_t = typename Dyn::RBDAcceleration_t;
    using RigidBodyAcceleration_t = typename Dyn::RigidBodyAcceleration_t;
    using JointAcceleration_t = typename Dyn::JointAcceleration_t;
    using ExtLinkForces_t = typename Dyn::ExtLinkForces_t;
    using EE_in_contact_t = typename Dyn::EE_in_contact_t;

    RBDState_t hyq_state;
    hyq_state.setDefault();

    control_vector_t torque_u = control_vector_t::Zero();
    ExtLinkForces_t ext_forces;
    ext_forces = ForceVector_t::Zero();

    RBDAcceleration_t hyq_xd;

    testdynamics.FloatingBaseForwardDynamics(hyq_state, torque_u, ext_forces, hyq_xd);

    RigidBodyAcceleration_t base_a;
    JointAcceleration_t qdd;
    testdynamics.FloatingBaseID(hyq_state, qdd, ext_forces, torque_u, base_a);

    ForceVector_t base_w;
    testdynamics.FloatingBaseFullyActuatedID(hyq_state, base_a, qdd, ext_forces, base_w, torque_u);

    EE_in_contact_t ee_contact = true;

    testdynamics.ProjectedForwardDynamics(ee_contact, hyq_state, torque_u, hyq_xd);

    testdynamics.ProjectedInverseDynamics(ee_contact, hyq_state, hyq_xd, torque_u);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
