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

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <ct/core/core.h>
#include <ct/rbd/rbd.h>

#include "ct/models/HyA/HyA.h"

using namespace ct;
using namespace ct::rbd;

#define Debug

TEST(HyaLinearizerTest, NumDiffComparison)
{
	typedef FixBaseFDSystem<HyA::Dynamics> HyASystem;

	const size_t STATE_DIM = HyASystem::STATE_DIM;
	const size_t CONTROL_DIM = HyASystem::CONTROL_DIM;

	std::shared_ptr<HyASystem > hyaSystem(new HyASystem);
	std::shared_ptr<HyASystem > hyaSystem2(new HyASystem);

	RbdLinearizer<HyASystem> rbdLinearizer(hyaSystem, true);
	core::SystemLinearizer<STATE_DIM, CONTROL_DIM> systemLinearizer(hyaSystem2, true);

	core::StateVector<STATE_DIM> x; x.setZero();
	core::ControlVector<CONTROL_DIM> u; u.setZero();

	auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 1.0);
	auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 1.0);

	auto A_system = systemLinearizer.getDerivativeState(x, u, 1.0);
	auto B_system = systemLinearizer.getDerivativeControl(x, u, 1.0);

	ASSERT_LT((A_rbd-A_system).array().abs().maxCoeff(),1e-5);
	ASSERT_LT((B_rbd-B_system).array().abs().maxCoeff(),1e-4);

	size_t nTests = 1000;
	for (size_t i=0; i<nTests; i++)
	{
		x.setRandom();
		u.setRandom();

		auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 0.0);
		auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 0.0);

		auto A_system = systemLinearizer.getDerivativeState(x, u, 0.0);
		auto B_system = systemLinearizer.getDerivativeControl(x, u, 0.0);

		ASSERT_LT((A_rbd-A_system).array().abs().maxCoeff(),1e-5);

		ASSERT_LT((B_rbd-B_system).array().abs().maxCoeff(),1e-4);
	}
}

TEST(CodegenLinearizerTest, NumDiffComparison)
{
	typedef FixBaseFDSystem<HyA::Dynamics> HyASystem;

	const size_t STATE_DIM = HyASystem::STATE_DIM;
	const size_t CONTROL_DIM = HyASystem::CONTROL_DIM;

	std::shared_ptr<HyASystem > hyaSystem(new HyASystem);

	RbdLinearizer<HyASystem> rbdLinearizer(hyaSystem, true);

	ct::models::HyA::HyALinearizedForward hyaLinear;

	core::StateVector<STATE_DIM> x;
	core::ControlVector<CONTROL_DIM> u;

	size_t nTests = 1000;
	for (size_t i=0; i<nTests; i++)
	{
		x.setRandom();
		u.setRandom();

		auto A_rbd = rbdLinearizer.getDerivativeState(x, u, 0.0);
		auto A_gen = hyaLinear.getDerivativeState(x, u, 0.0);

		auto B_rbd = rbdLinearizer.getDerivativeControl(x, u, 0.0);
		auto B_gen = hyaLinear.getDerivativeControl(x, u, 0.0);

		ASSERT_LT((A_rbd-A_gen).array().abs().maxCoeff(),1e-5);
		ASSERT_LT((B_rbd-B_gen).array().abs().maxCoeff(),1e-4);
	}
}


int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
