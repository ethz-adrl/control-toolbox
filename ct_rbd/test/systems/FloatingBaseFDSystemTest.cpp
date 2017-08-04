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

#include "../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct;
using namespace ct::rbd;

TEST(FloatingBaseFDSystemTest, forward_dynamics_test)
{
	const size_t STATE_DIM_QUAT = FloatingBaseFDSystem<TestHyQ::Dynamics, true>::STATE_DIM;
	const size_t STATE_DIM_EULER = FloatingBaseFDSystem<TestHyQ::Dynamics, false>::STATE_DIM;

	std::shared_ptr<core::System<STATE_DIM_QUAT> > dynamicsQuat(new FloatingBaseFDSystem<TestHyQ::Dynamics, true>);
	std::shared_ptr<core::System<STATE_DIM_EULER> > dynamicsEuler(new FloatingBaseFDSystem<TestHyQ::Dynamics, false>);

	core::Integrator<STATE_DIM_QUAT> integratorQuat(dynamicsQuat, ct::core::RK4);
	core::Integrator<STATE_DIM_EULER> integratorEuler(dynamicsEuler, ct::core::RK4);

	RBDState<12> state;
	state.setRandom();

	core::StateVector<STATE_DIM_QUAT> stateQuat = state.toStateVectorQuaternion();
	core::StateVector<STATE_DIM_EULER> stateEuler = state.toStateVectorEulerXyz();

	integratorQuat.integrate_n_steps(
			stateQuat,
			0,
			1000,
			0.001
	 );

	integratorEuler.integrate_n_steps(
			stateEuler,
				0,
				1000,
				0.001
	 );

	RBDState<12> finalQuat, finalEuler;
	finalQuat.fromStateVectorQuaternion(stateQuat);
	finalEuler.fromStateVectorEulerXyz(stateEuler);

	std::cout << "norm error between quat/euler integration: "<<std::endl<<(finalQuat.toStateVectorEulerXyz()-finalEuler.toStateVectorEulerXyz()).norm()<<std::endl;

	ASSERT_TRUE(finalQuat.isApprox(finalEuler, 1e-6));

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
