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

#include <gtest/gtest.h>

#include <ct/optcon/costfunction/CostFunctionAnalytical.hpp>
#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/term/TermQuadTracking.hpp>

#include <ct/optcon/costfunction/CostFunctionQuadraticTracking.hpp>


namespace ct{
namespace optcon{
namespace example{


template <size_t state_dim, size_t control_dim>
void compareCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction, CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
	ASSERT_NEAR(costFunction.evaluateIntermediate(), costFunction2.evaluateIntermediate(), 1e-7);

	ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(costFunction2.stateDerivativeIntermediate(), 1e-6));

	ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction2.stateSecondDerivativeIntermediate(), 1e-6));

	ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(costFunction2.controlDerivativeIntermediate(), 1e-6));

	ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(costFunction2.controlSecondDerivativeIntermediate(), 1e-6));

	ASSERT_TRUE(costFunction.stateControlDerivativeIntermediate().isApprox(costFunction2.stateControlDerivativeIntermediate(), 1e-6));

	// second derivatives have to be symmetric
	ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction.stateSecondDerivativeIntermediate().transpose(), 1e-6));
	ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(costFunction.controlSecondDerivativeIntermediate().transpose(), 1e-6));
}



TEST(CostFunctionTest, TrackingCfTest)
{
	// analytical costfunction
	std::shared_ptr<CostFunctionAnalytical<3, 3>> ANAcf (new CostFunctionAnalytical<3, 3>());

	Eigen::Matrix<double, 3, 3> Q;
	Eigen::Matrix<double, 3, 3> R;
	Q.setIdentity();
	R.setIdentity();

	core::StateTrajectory<3> stateTraj;
	core::ControlTrajectory<3> controlTraj;

	std::shared_ptr<CostFunctionQuadraticTracking<3, 3>> trackingCf (
			new CostFunctionQuadraticTracking<3,3>(Q, R, Q, core::InterpolationType::LIN, core::InterpolationType::ZOH, true));

	size_t trajSize = 5;

	bool timeIsAbsolute = true;

	for(size_t i = 0; i < trajSize; ++i)
	{
		stateTraj.push_back(core::StateVector<3>::Random(), double(i), timeIsAbsolute);
		controlTraj.push_back(core::ControlVector<3>::Random(), double(i), timeIsAbsolute);
	}

	std::shared_ptr< TermQuadTracking<3, 3> > term1 (new TermQuadTracking<3, 3>(Q, R, core::InterpolationType::LIN, core::InterpolationType::ZOH, true));

	term1->setStateAndControlReference(stateTraj, controlTraj);
	trackingCf->updateTrajectories(stateTraj, controlTraj);


	ANAcf->addIntermediateTerm(term1);

	Eigen::Vector3d x;
	Eigen::Vector3d u;

	x << -1.0, -2.0, 3.0;
	u << 1.0, 2.3, -3.1;

	double t = 0.0;

	ANAcf->setCurrentStateAndControl(x, u, t);
	trackingCf->setCurrentStateAndControl(x, u, t);

	ASSERT_TRUE(ANAcf->stateDerivativeIntermediateTest());
	ASSERT_TRUE(ANAcf->controlDerivativeIntermediateTest());

	ASSERT_TRUE(trackingCf->stateDerivativeIntermediateTest());
	ASSERT_TRUE(trackingCf->controlDerivativeIntermediateTest());

	compareCostFunctionOutput<3,3>(*ANAcf, *trackingCf);	
}


/*!
 * \example TrackingTest.cpp
 */
int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

} // namespace example
} // namespace optcon
} // namespace ct
