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

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
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

	std::shared_ptr<TestHyQ::tpl::Kinematics<valType>>  kyn(new TestHyQ::tpl::Kinematics<valType>);

	typedef TestHyQ::tpl::Dynamics<valType> Dyn;

	Dyn testdynamics(kyn);

	using control_vector_t 	= typename Dyn::control_vector_t;
	using ForceVector_t		= typename Dyn::ForceVector_t;
	using RBDState_t		= typename Dyn::RBDState_t;
	using RBDAcceleration_t = typename Dyn::RBDAcceleration_t;
	using RigidBodyAcceleration_t = typename Dyn::RigidBodyAcceleration_t;
	using JointAcceleration_t= typename Dyn::JointAcceleration_t;
	using ExtLinkForces_t		= typename Dyn::ExtLinkForces_t;
	using EE_in_contact_t = typename Dyn::EE_in_contact_t;

	RBDState_t hyq_state;
	hyq_state.setDefault();

	control_vector_t torque_u	= control_vector_t::Zero();
	ExtLinkForces_t ext_forces;
	ext_forces = ForceVector_t::Zero();

	RBDAcceleration_t hyq_xd;

	testdynamics.FloatingBaseForwardDynamics(hyq_state,torque_u, ext_forces, hyq_xd);

	RigidBodyAcceleration_t base_a;
	JointAcceleration_t qdd;
	testdynamics.FloatingBaseID(hyq_state,qdd,ext_forces,torque_u,base_a);

	ForceVector_t base_w;
	testdynamics.FloatingBaseFullyActuatedID(hyq_state,base_a,qdd,ext_forces,base_w,torque_u);

	EE_in_contact_t ee_contact=true;

	testdynamics.ProjectedForwardDynamics(ee_contact , hyq_state ,torque_u, hyq_xd );

	testdynamics.ProjectedInverseDynamics(ee_contact , hyq_state , hyq_xd , torque_u);

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
