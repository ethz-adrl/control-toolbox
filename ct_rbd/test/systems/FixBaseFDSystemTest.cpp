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
#include <memory>
#include <array>

#include <iostream>

#include <gtest/gtest.h>

#include "ct/rbd/systems/FixBaseFDSystem.h"

#include "../models/testIrb4600/RobCoGenTestIrb4600.h"

using namespace ct;
using namespace ct::rbd;

TEST(FixBaseFDSystemTest, forward_dynamics_test)
{
	const size_t STATE_DIM = FixBaseFDSystem<TestIrb4600::Dynamics>::STATE_DIM;

	std::shared_ptr<core::System<STATE_DIM> > dynamics(new FixBaseFDSystem<TestIrb4600::Dynamics>);

	core::Integrator<STATE_DIM> integrator(dynamics, ct::core::RK4);

	core::StateVector<12> state;
	state.setRandom();

	std::cout << "___________________________________________________________________________" << std::endl;

	std::cout << "Init state: " << state.transpose() << std::endl;

	integrator.integrate_n_steps(
			state,
			0,
			1000,
			0.001
	 );

	std::cout << "Integrated state: " << state.transpose() << std::endl;

	std::cout << "___________________________________________________________________________" << std::endl;

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
