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
	using RBDState_t = typename TestIrb4600::Dynamics::RBDState_t;
	using RBDAcceleration_t = typename TestIrb4600::Dynamics::RBDAcceleration_t;
	using JointState_t = typename TestIrb4600::Dynamics::JointState_t;
	using JointAcceleration_t = typename TestIrb4600::Dynamics::JointAcceleration_t;
	using ExtLinkForces_t = typename TestIrb4600::Dynamics::ExtLinkForces_t;
	using EE_in_contact_t = typename TestIrb4600::Dynamics::EE_in_contact_t;

	JointState_t irb_state;
	irb_state.setZero();

	control_vector_t torque_u = control_vector_t::Zero();
	ExtLinkForces_t ext_forces;
	ext_forces = ForceVector_t::Zero();

	JointAcceleration_t irb_xd;

	testdynamics.FixBaseForwardDynamics(irb_state, torque_u, ext_forces, irb_xd);

	JointAcceleration_t qdd;
	testdynamics.FixBaseID(irb_state, qdd, ext_forces, torque_u);


	// EE_in_contact_t ee_contact=true;

	// testdynamics.ProjectedForwardDynamics(ee_contact , irb_state ,torque_u, irb_xd );

	// testdynamics.ProjectedInverseDynamics(ee_contact , irb_state , irb_xd , torque_u);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
