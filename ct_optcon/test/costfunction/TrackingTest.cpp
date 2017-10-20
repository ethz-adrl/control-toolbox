/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus StÃ¤uble, Diego Pardo,
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

#include <gtest/gtest.h>

#include <ct/optcon/optcon.h>

namespace ct{
namespace optcon{
namespace example{


/*!
 * this is simply a little integration test that shows that the tracking cost function term builds and that
 * its analytic derivatives match NumDiff derivatives.
 */
TEST(TrackingTest, TrackingTermTest)
{
	const size_t state_dim = 12;
	const size_t control_dim = 4;

	// analytical costfunction
	std::shared_ptr<CostFunctionAnalytical<state_dim, control_dim>> costFunction (new CostFunctionAnalytical<state_dim, control_dim>());

	Eigen::Matrix<double, state_dim, state_dim> Q;
	Eigen::Matrix<double, control_dim, control_dim> R;
	Q.setIdentity();
	R.setIdentity();

	// create a reference trajectory and fill it with random values
	core::StateTrajectory<state_dim> stateTraj;
	core::ControlTrajectory<control_dim> controlTraj;
	size_t trajSize = 50;
	bool timeIsAbsolute = true;
	for(size_t i = 0; i < trajSize; ++i)
	{
		stateTraj.push_back(core::StateVector<state_dim>::Random(), double(i), timeIsAbsolute);
		controlTraj.push_back(core::ControlVector<control_dim>::Random(), double(i), timeIsAbsolute);
	}

	std::shared_ptr< TermQuadTracking<state_dim, control_dim> > trackingTerm (
			new TermQuadTracking<state_dim, control_dim>(Q, R, core::InterpolationType::LIN, core::InterpolationType::ZOH, true));

	trackingTerm->setStateAndControlReference(stateTraj, controlTraj);
	costFunction->addIntermediateTerm(trackingTerm);

	ct::core::StateVector<state_dim> x;
	ct::core::ControlVector<control_dim> u;
	x.setRandom();
	u.setRandom();
	double t = 0.0;

	costFunction->setCurrentStateAndControl(x, u, t);

	ASSERT_TRUE(costFunction->stateDerivativeIntermediateTest());
	ASSERT_TRUE(costFunction->controlDerivativeIntermediateTest());

}

} // namespace example
} // namespace optcon
} // namespace ct


/*!
 * This unit test tests the tracking cost function and the tracking cost function term.
 * \example TrackingTest.cpp
 */
int main(int argc, char** argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

