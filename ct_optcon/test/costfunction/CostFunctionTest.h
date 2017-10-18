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

#pragma once

#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DEBUG

#include <ct/optcon/optcon.h>
#include <gtest/gtest.h>

#include <cppad/example/cppad_eigen.hpp>

#include <ct/optcon/costfunction/CostFunctionAD.hpp>
#include <ct/optcon/costfunction/CostFunctionAnalytical.hpp>
#include <ct/optcon/costfunction/term/TermQuadratic.hpp>
#include <ct/optcon/costfunction/term/TermQuadMult.hpp>
#include <ct/optcon/costfunction/term/TermMixed.hpp>

const size_t state_dim = 12;
const size_t control_dim = 4;

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


template <size_t state_dim, size_t control_dim>
void printCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction, CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
	std::cout << "eval intermediate " << std::endl;
	std::cout << costFunction.evaluateIntermediate() << std::endl << std::endl;
	std::cout << costFunction2.evaluateIntermediate() << std::endl;

	std::cout << "eval terminal " << std::endl;
	std::cout << costFunction.evaluateTerminal() << std::endl << std::endl;
	std::cout << costFunction2.evaluateTerminal() << std::endl;

	std::cout << "eval stateDerivativeIntermediate " << std::endl;
	std::cout << costFunction.stateDerivativeIntermediate()<< std::endl << std::endl;
	std::cout << costFunction2.stateDerivativeIntermediate() << std::endl;

	std::cout << "eval stateDerivativeIntermediate " << std::endl;
	std::cout << costFunction.stateDerivativeTerminal() << std::endl << std::endl;
	std::cout << costFunction2.stateDerivativeTerminal() << std::endl;

	std::cout << "eval stateSecondDerivativeIntermediate " << std::endl;
	std::cout << costFunction.stateSecondDerivativeIntermediate() << std::endl << std::endl;
	std::cout << costFunction2.stateSecondDerivativeIntermediate() << std::endl;

	std::cout << "eval stateSecondDerivativeTerminal " << std::endl;
	std::cout << costFunction.stateSecondDerivativeTerminal() << std::endl << std::endl;
	std::cout << costFunction2.stateSecondDerivativeTerminal() << std::endl;

	std::cout << "eval controlDerivativeIntermediate " << std::endl;
	std::cout << costFunction.controlDerivativeIntermediate() << std::endl << std::endl;
	std::cout << costFunction2.controlDerivativeIntermediate() << std::endl;

	std::cout << "eval controlDerivativeTerminal " << std::endl;
	std::cout << costFunction.controlDerivativeTerminal() << std::endl << std::endl;
	std::cout << costFunction2.controlDerivativeTerminal() << std::endl;

	std::cout << "eval controlSecondDerivativeIntermediate " << std::endl;
	std::cout << costFunction.controlSecondDerivativeIntermediate() << std::endl << std::endl;
	std::cout << costFunction2.controlSecondDerivativeIntermediate() << std::endl;

	std::cout << "eval controlSecondDerivativeIntermediate " << std::endl;
	std::cout << costFunction.controlSecondDerivativeTerminal() << std::endl << std::endl;
	std::cout << costFunction2.controlSecondDerivativeTerminal() << std::endl;

	std::cout << "eval stateControlDerivativeIntermediate " << std::endl;
	std::cout << costFunction.stateControlDerivativeIntermediate() << std::endl << std::endl;
	std::cout << costFunction2.stateControlDerivativeIntermediate() << std::endl;

	std::cout << "eval stateControlDerivativeTerminal " << std::endl;
	std::cout << costFunction.stateControlDerivativeTerminal() << std::endl << std::endl;
	std::cout << costFunction2.stateControlDerivativeTerminal() << std::endl;

	std::cout << "eval stateSecondDerivativeIntermediate " << std::endl;
	std::cout << costFunction.stateSecondDerivativeIntermediate() << std::endl << std::endl;
	std::cout << costFunction.stateSecondDerivativeIntermediate().transpose() << std::endl;

	std::cout << "eval controlSecondDerivativeIntermediate " << std::endl;
	std::cout << costFunction.controlSecondDerivativeIntermediate() << std::endl << std::endl;
	std::cout << costFunction.controlSecondDerivativeIntermediate().transpose() << std::endl;
}


TEST(CostFunctionTest, ADQuadraticTest)
{
	const size_t nWeights = 2;
	const size_t nTests = 10;

	CostFunctionAnalytical<state_dim, control_dim> costFunction;
	CostFunctionAD<state_dim, control_dim> costFunctionAD;

	// intermediate cost terms
	std::shared_ptr<TermQuadratic<state_dim, control_dim, double> > termQuadratic_interm(new TermQuadratic<state_dim, control_dim>);
	std::shared_ptr<TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar > > termQuadraticAD_interm (	new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

	// final cost terms
	std::shared_ptr<TermQuadratic<state_dim, control_dim, double> > termQuadratic_final (new TermQuadratic<state_dim, control_dim>);
	std::shared_ptr<TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar > > termQuadraticAD_final (	new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

	costFunction.addIntermediateTerm(termQuadratic_interm, true);
	costFunctionAD.addIntermediateADTerm(termQuadraticAD_interm, true);
	costFunction.addFinalTerm(termQuadratic_final, true);
	costFunctionAD.addFinalADTerm(termQuadraticAD_final, true);

	Eigen::Matrix<double, state_dim, state_dim> Q_interm;
	Eigen::Matrix<double, state_dim, state_dim> Q_final;
	Eigen::Matrix<double, control_dim, control_dim> R;

	core::StateVector<state_dim> x_ref;
	core::ControlVector<control_dim> u_ref;

	for (size_t i=0; i<nWeights; i++)
	{
		try{
		Q_interm.setRandom();
		Q_final.setRandom();
		R.setRandom();
		x_ref.setRandom();
		u_ref.setRandom();

		if (i==0)
		{
			Q_interm.setZero();
			Q_final.setZero();
			R.setZero();
			x_ref.setZero();
			u_ref.setZero();
		}

		Q_interm += Q_interm.transpose().eval(); // make symmetric
		R += R.transpose().eval(); // make symmetric
		Q_final += Q_final.transpose().eval(); // make symmetric

		termQuadratic_interm->setWeights(Q_interm, R);
		termQuadraticAD_interm->setWeights(Q_interm, R);
		termQuadratic_interm->setStateAndControlReference(x_ref, u_ref);
		termQuadraticAD_interm->setStateAndControlReference(x_ref, u_ref);

		termQuadratic_final->setWeights(Q_final, R);
		termQuadraticAD_final->setWeights(Q_final, R);
		termQuadratic_final->setStateAndControlReference(x_ref, u_ref);
		termQuadraticAD_final->setStateAndControlReference(x_ref, u_ref);

		costFunctionAD.initialize();

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

			costFunction.setCurrentStateAndControl(x, u, 1.0);
			costFunctionAD.setCurrentStateAndControl(x, u, 1.0);

//			printCostFunctionOutput(costFunction, costFunctionAD);
			compareCostFunctionOutput(costFunction, costFunctionAD);

			// now some manual assertions
			ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(2*Q_interm*(x-x_ref)));
			ASSERT_TRUE(costFunction.stateDerivativeTerminal().isApprox(2*Q_final * (x-x_ref)));

			ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(2*Q_interm));
			ASSERT_TRUE(costFunction.stateSecondDerivativeTerminal().isApprox(2*Q_final));

			ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(2*R*(u-u_ref)));

			ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(2*R));
		}
		}
		catch(std::exception& e)
		{
			FAIL();
		}
	}
}


TEST(CostFunctionTest, ADQuadMultTest)
{
	const size_t nWeights = 3;
	const size_t nTests = 10;

	CostFunctionAnalytical<state_dim, control_dim> costFunction;
	CostFunctionAD<state_dim, control_dim> costFunctionAD;

	std::shared_ptr<TermQuadMult<state_dim, control_dim, double> > termQuadMult(new TermQuadMult<state_dim, control_dim>);
	std::shared_ptr<TermQuadMult<state_dim, control_dim, double, ct::core::ADCGScalar > > termQuadMultAD(new TermQuadMult<state_dim, control_dim, double, ct::core::ADCGScalar>);

	std::shared_ptr<TermMixed<state_dim, control_dim, double > > termMixed (new TermMixed<state_dim, control_dim, double>);
	std::shared_ptr<TermMixed<state_dim, control_dim, double, ct::core::ADCGScalar > > termMixedAD (new TermMixed<state_dim, control_dim, double, ct::core::ADCGScalar>);


	costFunction.addIntermediateTerm(termQuadMult);
	costFunctionAD.addIntermediateADTerm(termQuadMultAD);

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

			Q += Q.transpose().eval(); // make symmetric
			R += R.transpose().eval(); // make symmetric

			termQuadMult->setWeights(Q, R);
			termQuadMultAD->setWeights(Q, R);
			termQuadMult->setStateAndControlReference(x_ref, u_ref);
			termQuadMultAD->setStateAndControlReference(x_ref, u_ref);

			costFunctionAD.initialize();

			// create cloned cost function
			std::shared_ptr<CostFunctionAD<state_dim, control_dim>> costFunctionAD_clone (costFunctionAD.clone());
			costFunctionAD_clone->initialize();


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

