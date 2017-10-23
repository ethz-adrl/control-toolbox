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

#include <gtest/gtest.h>

#include <ct/rbd/rbd.h>

#include <ct/rbd/robot/kinematics/RBDDataMap.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;

TEST(DataMapTestsHyQ, datamap_test_bool)
{
	RBDDataMap<bool, TestHyQ::RobCoGenContainer::NLINKS> ldm_a;
	RBDDataMap<bool, TestHyQ::RobCoGenContainer::NLINKS> ldm_b;

	for (size_t id = 0; id < TestHyQ::RobCoGenContainer::NLINKS; ++id)
	{
		ldm_a[id] = true;
		ldm_b[id] = false;
	}

	// check all values
	for (size_t id = 0; id < TestHyQ::RobCoGenContainer::NLINKS; ++id)
	{
		ASSERT_TRUE(ldm_a[id] == true);
		ASSERT_TRUE(ldm_b[id] == false);
		ldm_a[id] = ldm_b[id];
		ASSERT_FALSE(ldm_a[id] == true);
	}

	// constant assignment
	ldm_b = true;
	ASSERT_TRUE(ldm_b[0] == true);
	ASSERT_TRUE(ldm_b[TestHyQ::RobCoGenContainer::NLINKS - 1] == true);

	// copy from other
	ldm_b = ldm_a;
	ASSERT_TRUE(ldm_b[0] == false);
	ASSERT_TRUE(ldm_b[TestHyQ::RobCoGenContainer::NLINKS - 1] == false);

	// << operator
	std::cout << ldm_b << std::endl;
}

TEST(DataMapTestsHyQ, datamap_test_eigen)
{
	const size_t numEE = TestHyQ::Kinematics::NUM_EE;

	RBDDataMap<Eigen::Vector3d, numEE> ldm_a;
	RBDDataMap<Eigen::Vector3d, numEE> ldm_b;

	for (size_t id = 0; id < numEE; ++id)
	{
		ldm_a[id] = Eigen::Vector3d::Constant(-888.8);
		ldm_b[id] = Eigen::Vector3d::Constant(222.2);
	}

	// check all values
	for (size_t id = 0; id < numEE; ++id)
	{
		ASSERT_TRUE(ldm_a[id](1) == -888.8);
		ASSERT_TRUE(ldm_b[id](1) == 222.2);
		ldm_a[id] = ldm_b[id];
		ASSERT_FALSE(ldm_a[id](1) == -888.8);
	}

	// constant assignment
	ldm_b = Eigen::Vector3d::Constant(-888.8);
	ASSERT_TRUE(ldm_b[0](1) == -888.8);
	ASSERT_TRUE(ldm_b[numEE - 1](1) == -888.8);

	// copy from other
	ldm_b = ldm_a;
	ASSERT_TRUE(ldm_b[0](1) == 222.2);
	ASSERT_TRUE(ldm_b[numEE - 1](1) == 222.2);

	// << operator
	std::cout << ldm_b << std::endl;
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
