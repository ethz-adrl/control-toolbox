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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#include <ct/core/core.h>

using namespace ct::core;

void testSensitivities(IntegrationType integrationType)
{
	std::shared_ptr<ControlledSystem<2, 1>> system(new SecondOrderSystem(10));
	std::shared_ptr<ConstantController<2,1>> controller(new ConstantController<2,1>);
	system->setController(controller);
	controller->setControl(ControlVector<1>::Random());

	std::shared_ptr<SubstepRecorder<2,1>> substepRecorder(new SubstepRecorder<2,1>(system));
	Integrator<2> integrator(system, integrationType, substepRecorder);

	std::shared_ptr<SystemLinearizer<2,1>> linearSystem(new SystemLinearizer<2,1>(system));

	typedef std::shared_ptr<Sensitivity<2,1>> SensitivityPtr;
	std::vector<SensitivityPtr, Eigen::aligned_allocator<SensitivityPtr>> sensitivities;

	double dt = 0.01;

	sensitivities.push_back(SensitivityPtr(new SensitivityApproximation<2,1>(dt, linearSystem)));
	sensitivities.push_back(SensitivityPtr(new SensitivityIntegrator<2,1>(dt, linearSystem, controller)));


	size_t kSteps = 5;

	StateVector<2> state; state.setRandom();

	typedef std::shared_ptr<StateVectorArray<2>> StateVectorArrayPtr;
	typedef std::shared_ptr<ControlVectorArray<1>> ControlVectorArrayPtr;
	std::vector<StateVectorArrayPtr, Eigen::aligned_allocator<StateVectorArrayPtr>> xSubstep(kSteps);
	std::vector<ControlVectorArrayPtr, Eigen::aligned_allocator<ControlVectorArrayPtr>> uSubstep(kSteps);

	for (size_t j=0; j<sensitivities.size(); j++)
	{
		sensitivities[j]->setSubstepTrajectoryReference(&xSubstep, &uSubstep);
	}

	for(size_t i=0; i<kSteps; i++)
	{
		StateVector<2> stateBefore = state;

		substepRecorder->reset();
		integrator.integrate_n_steps(
				state,
				0.0,
				1,
				dt);

		for (size_t j=0; j<sensitivities.size(); j++)
		{
			xSubstep[i] = substepRecorder->getSubstates();
			uSubstep[i] = substepRecorder->getSubcontrols();

			StateMatrix<2> A;
			StateControlMatrix<2,1> B;

			sensitivities[j]->getAandB(stateBefore,
				controller->getControl(),
				state,
				i,
				1,
				A,
				B);

			std::cout << "A:" << std::endl << A <<std::endl << std::endl;
			std::cout << "B:" << std::endl << B <<std::endl << std::endl;

			std::cout <<std::endl << std::endl;
		}
	}
}

int main(int argc, char* argv[])
{
	testSensitivities(IntegrationType::EULERCT);
	testSensitivities(IntegrationType::RK4CT);

	return 1;
}
