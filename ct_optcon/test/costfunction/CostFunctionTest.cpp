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

#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DEBUG

#include <gtest/gtest.h>

#include <cppad/example/cppad_eigen.hpp>

#include <ct/optcon/costfunction/CostFunctionAD.hpp>
#include <ct/optcon/costfunction/CostFunctionAnalytical.hpp>
#include <ct/optcon/costfunction/term/TermQuadratic.hpp>
#include <ct/optcon/costfunction/term/TermQuadMult.hpp>
#include <ct/optcon/costfunction/term/TermMixed.hpp>

const size_t state_dim = 16;
const size_t control_dim = 14;

namespace ct{
namespace optcon{
namespace example{


template <size_t state_dim, size_t control_dim>
void compareCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction, CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
	ASSERT_NEAR(costFunction.evaluateIntermediate(), costFunction2.evaluateIntermediate(), 1e-9);
	ASSERT_NEAR(costFunction.evaluateTerminal(), costFunction2.evaluateTerminal(), 1e-9);

	ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(costFunction2.stateDerivativeIntermediate()));
	ASSERT_TRUE(costFunction.stateDerivativeTerminal().isApprox(costFunction2.stateDerivativeTerminal()));

	ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction2.stateSecondDerivativeIntermediate()));
	ASSERT_TRUE(costFunction.stateSecondDerivativeTerminal().isApprox(costFunction2.stateSecondDerivativeTerminal()));

	ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(costFunction2.controlDerivativeIntermediate()));
	ASSERT_TRUE(costFunction.controlDerivativeTerminal().isApprox(costFunction2.controlDerivativeTerminal()));

	ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(costFunction2.controlSecondDerivativeIntermediate()));
	ASSERT_TRUE(costFunction.controlSecondDerivativeTerminal().isApprox(costFunction2.controlSecondDerivativeTerminal()));

	ASSERT_TRUE(costFunction.stateControlDerivativeIntermediate().isApprox(costFunction2.stateControlDerivativeIntermediate()));
	ASSERT_TRUE(costFunction.stateControlDerivativeTerminal().isApprox(costFunction2.stateControlDerivativeTerminal()));

	// second derivatives have to be symmetric
	ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction.stateSecondDerivativeIntermediate().transpose()));
	ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(costFunction.controlSecondDerivativeIntermediate().transpose()));
}

TEST(CostFunctionTest, ADQuadraticIntermediateTest)
{
	const size_t nWeights = 10;
	const size_t nTests = 10;

	CostFunctionAnalytical<state_dim, control_dim> costFunction;
	CostFunctionAD<state_dim, control_dim> costFunctionAD;

	std::shared_ptr<TermQuadratic<state_dim, control_dim, double> > termQuadratic(new TermQuadratic<state_dim, control_dim>);
	std::shared_ptr<TermQuadratic<state_dim, control_dim, CppAD::AD<double>, double > > termQuadraticAD(new TermQuadratic<state_dim, control_dim, CppAD::AD<double>, double>);

	double t_on = 0.5;
	double t_off = 1.5;

	double t_final = 2.5;

	std::shared_ptr<SingleActivation> c_single (new SingleActivation(t_on, t_off));

	termQuadratic->setTimeActivation(c_single, true);
	termQuadraticAD->setTimeActivation(c_single, true);

	costFunction.addIntermediateTerm(termQuadratic, true);
	size_t termIdAD = costFunctionAD.addIntermediateTerm(termQuadraticAD, true);

	Eigen::Matrix<double, state_dim, state_dim> Q;
	Eigen::Matrix<double, control_dim, control_dim> R;

	core::StateVector<state_dim> x_ref;
	core::ControlVector<control_dim> u_ref;

	for (size_t i=0; i<nWeights; i++)
	{
		try{
		Q.setRandom();
		R.setRandom();
		x_ref.setRandom();
		u_ref.setRandom();

		if (i==0)
		{
			Q.setZero();
			R.setZero();
			x_ref.setZero();
			u_ref.setZero();
		}

		termQuadratic->setWeights(Q, R);
		termQuadraticAD->setWeights(Q, R);
		termQuadratic->setStateAndControlReference(x_ref, u_ref);
		termQuadraticAD->setStateAndControlReference(x_ref.template cast<CppAD::AD<double>>(), u_ref.template cast<CppAD::AD<double>>());

		costFunctionAD.termChanged(termIdAD);

		for (size_t j=0; j<nTests; j++)
		{
			core::StateVector<state_dim> x;
			core::ControlVector<control_dim> u;
			x.setRandom();
			u.setRandom();

			if (j==0)
			{
				x.setZero();
				u.setZero();
			}

			costFunction.setCurrentStateAndControl(x, u, 0.0);
			costFunctionAD.setCurrentStateAndControl(x, u, 0.0);

			compareCostFunctionOutput(costFunction, costFunctionAD);
		}
		}
		catch(std::exception& e)
		{
			FAIL();
		}
	}
}

TEST(CostFunctionTest, ADQuadMultIntermediateTest)
{
	const size_t nWeights = 10;
	const size_t nTests = 10;

	CostFunctionAnalytical<state_dim, control_dim> costFunction;
	CostFunctionAD<state_dim, control_dim> costFunctionAD;

	std::shared_ptr<TermQuadMult<state_dim, control_dim, double> > termQuadMult(new TermQuadMult<state_dim, control_dim>);
	std::shared_ptr<TermQuadMult<state_dim, control_dim, CppAD::AD<double>, double > > termQuadMultAD(new TermQuadMult<state_dim, control_dim, CppAD::AD<double>, double>);

	std::shared_ptr<TermMixed<state_dim, control_dim, double > > termMixed (new TermMixed<state_dim, control_dim, double>);
	std::shared_ptr<TermMixed<state_dim, control_dim, CppAD::AD<double> > > termMixedAD (new TermMixed<state_dim, control_dim, CppAD::AD<double>>);

	double active_percentage = 0.5; // how much of the cycle is the time active
	double period = 0.5; // what is the period
	double activation_offset = 0.1; // how much is the activation offset WITHIN the period
	double period_offset = 0.2; // how much is the period offset to t=0?

	double t_final = 2.5;

	std::shared_ptr<PeriodicActivation> c_periodic (new PeriodicActivation(active_percentage, period, activation_offset, period_offset));
	termQuadMult->setTimeActivation(c_periodic, true);
	termQuadMultAD->setTimeActivation(c_periodic, true);

	costFunction.addIntermediateTerm(termQuadMult);
	size_t termIdAD = costFunctionAD.addIntermediateTerm(termQuadMultAD);

	Eigen::Matrix<double, state_dim, state_dim> Q;
	Eigen::Matrix<double, control_dim, control_dim> R;
	Eigen::Matrix<double, control_dim, state_dim> P;

	core::StateVector<state_dim> x_ref;
	core::ControlVector<control_dim> u_ref;

	for (size_t i=0; i<nWeights; i++)
	{
		try{
			Q.setRandom();
			R.setRandom();
			P.setRandom();
			x_ref.setRandom();
			u_ref.setRandom();

			if (i==0)
			{
				Q.setZero();
				R.setZero();
				P.setZero();
				x_ref.setZero();
				u_ref.setZero();
			}

			if (i==1)
			{
				Q.setIdentity();
				R.setIdentity();
				P.setZero();
				x_ref.setConstant(10.0);
				u_ref.setConstant(10.0);
			}

			termQuadMult->setWeights(Q, R);
			termQuadMultAD->setWeights(Q, R);
			termQuadMult->setStateAndControlReference(x_ref, u_ref);
			termQuadMultAD->setStateAndControlReference(x_ref, u_ref);

			costFunctionAD.termChanged(termIdAD);

			// create cloned cost function
			std::shared_ptr<CostFunctionAD<state_dim, control_dim>> costFunctionAD_clone (costFunctionAD.clone());


			for (size_t j=0; j<nTests; j++)
			{
				core::StateVector<state_dim> x;
				core::ControlVector<control_dim> u;
				x.setRandom();
				u.setRandom();
				double t = 3.14;

				if (j==0)
				{
					x.setZero();
					u.setZero();
				}

				costFunction.setCurrentStateAndControl(x, u, t);
				costFunctionAD.setCurrentStateAndControl(x, u, t);
				costFunctionAD_clone->setCurrentStateAndControl(x, u, t);

				compareCostFunctionOutput(costFunction, costFunctionAD);
				compareCostFunctionOutput(*costFunctionAD_clone, costFunctionAD);
			}
		}
		catch(std::exception& e) {
			FAIL();
		}
	}
}


} // namespace example
} // namespace optcon
} // namespace ct

/*!
 * This unit test illustrates the use of cost functions and terms
 * \example CostFunctionTest.cpp
 */
int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

