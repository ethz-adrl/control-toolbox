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

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <gtest/gtest.h>

#include <ct/rbd/state/JointState.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"
#include "../../../include/ct/rbd/robot/Kinematics.h"

#include "ct/rbd/robot/kinematics/EndEffector.h"

using namespace ct;
using namespace rbd;


TEST(TestHyQKinematicsAd, transformTest)
{
	typedef float size_type;
	typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;
	KinTpl_t kynTpl;

	tpl::JointState<TestHyQ::tpl::Kinematics<size_type>::NJOINTS, size_type> hyqJointState;
	tpl::RigidBodyPose<size_type> hyqPose;
	Eigen::Matrix<size_type, 3, 1> vec3Tpl;

	hyqPose.setRandom();
	vec3Tpl.setRandom();

	size_t ind = 1;

	kindr::Position<size_type, 3> pos = kynTpl.getEEPositionInWorld(ind, hyqPose, hyqJointState.getPositions());
	auto force3d = kynTpl.mapForceFromWorldToLink3d(vec3Tpl, hyqPose, hyqJointState.getPositions(), ind);
	auto forceLink = kynTpl.mapForceFromWorldToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);
	auto forceEELink =  kynTpl.mapForceFromEEToLink(force3d, hyqPose, hyqJointState.getPositions(), ind);

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



