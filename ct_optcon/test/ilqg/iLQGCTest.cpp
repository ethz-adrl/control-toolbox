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

#include <chrono>

// Bring in gtest
#include <gtest/gtest.h>

//#define DEBUG_PRINT
//#define DEBUG_PRINT_LINESEARCH

#include <ct/optcon/optcon.h>


/* This test implements a 1-Dimensional horizontally moving point mass with mass 1kg and attached to a spring
 x = [p, pd] // p - position pointing upwards, against gravity, pd - velocity
 dx = f(x,u)
    = [0 1  x  +  [0      +  [0  u
       0 0]        9.81]      1]

 */


namespace ct{
namespace optcon{
namespace example{

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;

const size_t state_dim = 2; // position, velocity
const size_t control_dim = 1; // force

const double kStiffness = 10;

//! Dynamics class for the iLQG unit test
class Dynamics : public ControlledSystem<state_dim, control_dim>
{
public:
	Dynamics() : ControlledSystem<state_dim, control_dim>(SYSTEM_TYPE::SECOND_ORDER) {}

	void computeControlledDynamics(
			const StateVector<state_dim>& state,
			const Time& t,
			const ControlVector<control_dim>& control,
			StateVector<state_dim>& derivative
	) override
	{
		derivative(0) = state(1);
		derivative(1) = control(0) - kStiffness*state(0); // mass is 1 kg
	}

	Dynamics* clone() const override
	{
		return new Dynamics();
	};
};

//! Linear system class for the iLQG unit test
class LinearizedSystem : public LinearSystem<state_dim, control_dim>
{
public:
	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x, const ControlVector<control_dim>& u, const double t = 0.0) override
			{
		A_ << 0, 1, -kStiffness, 0;
		return A_;
			}

	const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x, const ControlVector<control_dim>& u, const double t = 0.0) override
			{

		B_ << 0, 1;
		return B_;
			}

	LinearizedSystem* clone() const override
			{
		return new LinearizedSystem();
			};
};

//! Create a cost function for the iLQG unit test
std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > createCostFunction(Eigen::Vector2d& x_final)
{
	Eigen::Matrix2d Q;
	Q << 0, 0, 0, 1;

	Eigen::Matrix<double, 1, 1> R;
	R << 100.0;

	Eigen::Vector2d x_nominal = Eigen::Vector2d::Zero();
	Eigen::Matrix<double, 1, 1> u_nominal = Eigen::Matrix<double, 1, 1>::Zero();

	Eigen::Matrix2d Q_final;
	Q_final << 10, 0, 0, 10;

	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > quadraticCostFunction(
			new CostFunctionQuadraticSimple<state_dim, control_dim>(
					Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}


TEST(ILQCTest, SystemLinearizationTest)
{
	shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
	shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
	shared_ptr<LinearSystem<state_dim, control_dim> > numDiffLinearModelGeneral(new SystemLinearizer<state_dim, control_dim>(nonlinearSystem, false));
	shared_ptr<LinearSystem<state_dim, control_dim> > numDiffLinearModelSecondOrder(new SystemLinearizer<state_dim, control_dim>(nonlinearSystem, true));
	StateVector<state_dim> xRef;
	ControlVector<control_dim> u;
	Time t=0;

	size_t nTests = 100;
	for (size_t i=0; i<nTests; i++)
	{
		xRef.setRandom();

		// we have to set u in a way that it makes x and equilibrium
		u(0) = kStiffness*xRef(0);

		StateVector<state_dim> dxNonlinear;
		nonlinearSystem->computeControlledDynamics(xRef, t, u, dxNonlinear);

		StateVector<state_dim> dxLinear;
		analyticLinearSystem->computeControlledDynamics(xRef, t, u, dxLinear);

		StateVector<state_dim> dxLinearNumDiffGeneral;
		numDiffLinearModelGeneral->computeControlledDynamics(xRef, t, u, dxLinearNumDiffGeneral);

		StateVector<state_dim> dxLinearNumDiffSecondOrder;
		numDiffLinearModelGeneral->computeControlledDynamics(xRef, t, u, dxLinearNumDiffSecondOrder);

		ASSERT_LT(
				(dxNonlinear-dxLinear).array().abs().maxCoeff(),
				1e-6
		);

		ASSERT_LT(
				(dxNonlinear-dxLinearNumDiffGeneral).array().abs().maxCoeff(),
				1e-6
		);

		ASSERT_LT(
				(dxNonlinear-dxLinearNumDiffSecondOrder).array().abs().maxCoeff(),
				1e-6
		);
	}
}

TEST(ILQCTest, SingleCoreTest)
{
	try {

		std::cout << "setting up problem " << std::endl;

		Eigen::Vector2d x_final;
		x_final << 20, 0;

		iLQGSettings ilqg_settings;
		ilqg_settings.epsilon = 0.0;
		ilqg_settings.nThreads = 4;
		ilqg_settings.max_iterations = 50;
		ilqg_settings.recordSmallestEigenvalue = true;
		ilqg_settings.min_cost_improvement = 1e-6;

		shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
		shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
		shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

		// times
		ct::core::Time tf = 3.0;

		// init state
		StateVector<state_dim> x0;  x0.setRandom();

		// construct single-core single subsystem OptCon Problem
		OptConProblem<state_dim, control_dim> optConProblem (tf, x0, nonlinearSystem, costFunction, analyticLinearSystem);

		size_t nSteps = std::round(tf / ilqg_settings.dt);

		std::cout << "initializing ilqg solver" << std::endl;
		iLQG<state_dim, control_dim> ilqg(optConProblem, ilqg_settings);
		iLQGMP<state_dim, control_dim> ilqg_mp(optConProblem, ilqg_settings);


		// provide initial controller
		FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
		ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
		iLQG<state_dim, control_dim>::Policy_t initController (u0_ff, u0_fb, ilqg_settings.dt);

		ilqg.configure(ilqg_settings);
		ilqg.setInitialGuess(initController);


		ilqg.solve();


		size_t nTests = 4;
		for (size_t i=0; i<nTests; i++)
		{
			if (i==0)
				ilqg_settings.lineSearchSettings.active = false;
			else
				ilqg_settings.lineSearchSettings.active = true;

			if (i<2)
				ilqg_settings.fixedHessianCorrection = false;
			else
				ilqg_settings.fixedHessianCorrection = true;

			ilqg.configure(ilqg_settings);
			ilqg_mp.configure(ilqg_settings);

			ilqg.setInitialGuess(initController);
			ilqg_mp.setInitialGuess(initController);

			bool foundBetter = true;
			bool foundBetter_mp = true;
			size_t numIterations = 0;

			while (foundBetter)
			{
				// solve

				foundBetter = ilqg.runIteration();
				foundBetter_mp = ilqg_mp.runIteration();

				// test trajectories

				StateTrajectory<state_dim> xRollout = ilqg.getStateTrajectory();
				ControlTrajectory<control_dim> uRollout = ilqg.getControlTrajectory();

				StateTrajectory<state_dim> xRollout_mp = ilqg_mp.getStateTrajectory();
				ControlTrajectory<control_dim> uRollout_mp = ilqg_mp.getControlTrajectory();

				ASSERT_EQ(xRollout.size(), nSteps+1);
				ASSERT_EQ(uRollout.size(), nSteps);

				ASSERT_EQ(xRollout_mp.size(), nSteps+1);
				ASSERT_EQ(uRollout_mp.size(), nSteps);


				// test linearization

				iLQG<state_dim, control_dim>::StateMatrixArray A;
				iLQG<state_dim, control_dim>::StateControlMatrixArray B;
				ilqg.retrieveLastLinearizedModel(A, B);

				iLQGMP<state_dim, control_dim>::StateMatrixArray A_mp;
				iLQGMP<state_dim, control_dim>::StateControlMatrixArray B_mp;
				ilqg_mp.retrieveLastLinearizedModel(A_mp, B_mp);

				ASSERT_EQ(A.size(), nSteps);
				ASSERT_EQ(B.size(), nSteps);

				ASSERT_EQ(A_mp.size(), nSteps);
				ASSERT_EQ(B_mp.size(), nSteps);

				// check integration
				for (size_t j=0; j<xRollout.size()-1; j++)
				{
					ASSERT_LT((xRollout[j] - xRollout_mp[j]).array().abs().maxCoeff(), 1e-10);
					ASSERT_LT((uRollout[j] - uRollout_mp[j]).array().abs().maxCoeff(), 1e-10);
				}

				// check linearization
				for (size_t j=0; j<xRollout.size()-1; j++)
				{
					LinearizedSystem::state_matrix_t A_analytic;
					LinearizedSystem::state_control_matrix_t B_analytic;

					if(ilqg_settings.discretization == iLQGSettings::FORWARD_EULER)
					{
						A_analytic = LinearizedSystem::state_matrix_t::Identity() + ilqg_settings.dt * analyticLinearSystem->getDerivativeState(xRollout[j], uRollout[j], 0);
						B_analytic = ilqg_settings.dt * analyticLinearSystem->getDerivativeControl(xRollout[j], uRollout[j], 0);	
					}
					else if(ilqg_settings.discretization == iLQGSettings::BACKWARD_EULER)
					{
						LinearizedSystem::state_matrix_t aNew = ilqg_settings.dt * analyticLinearSystem->getDerivativeState(xRollout[j], uRollout[j], 0);
						LinearizedSystem::state_matrix_t aNewInv = (LinearizedSystem::state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
						A_analytic = aNewInv;
						B_analytic = aNewInv * ilqg_settings.dt * analyticLinearSystem->getDerivativeControl(xRollout[j], uRollout[j], 0);
					}
					else if(ilqg_settings.discretization == iLQGSettings::TUSTIN)
					{
						LinearizedSystem::state_matrix_t aNew = 0.5 * ilqg_settings.dt * analyticLinearSystem->getDerivativeState(xRollout[j], uRollout[j], 0);
						LinearizedSystem::state_matrix_t aNewInv = (LinearizedSystem::state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
						A_analytic = aNewInv * (LinearizedSystem::state_matrix_t::Identity() + aNew);
						B_analytic = aNewInv * ilqg_settings.dt * analyticLinearSystem->getDerivativeControl(xRollout[j], uRollout[j], 0);
					}

					ASSERT_LT(
							(A[j]-A_analytic).array().abs().maxCoeff(),
							1e-6
					);
					ASSERT_LT(
							(A_mp[j]-A_analytic).array().abs().maxCoeff(),
							1e-6
					);
					ASSERT_LT(
							(A_mp[j]-A[j]).array().abs().maxCoeff(),
							1e-12
					);

					ASSERT_LT(
							(B[j]-B_analytic).array().abs().maxCoeff(),
							1e-6
					);
					ASSERT_LT(
							(B_mp[j]-B_analytic).array().abs().maxCoeff(),
							1e-6
					);
					ASSERT_LT(
							(B_mp[j]-B[j]).array().abs().maxCoeff(),
							1e-12
					);
				}

				ASSERT_EQ(foundBetter, foundBetter_mp);

				numIterations++;

				// we should converge in way less than 20 iterations
				ASSERT_LT(numIterations, 20);
			}

			// make sure we are really converged
			//ASSERT_FALSE(ilqg.runIteration());

			// make sure we are really converged
			//ASSERT_FALSE(ilqg_mp.runIteration());
		}

	} catch (std::exception& e)
	{
		std::cout << "caught exception: "<<e.what() <<std::endl;
		FAIL();
	}
}

TEST(ILQCTest, InstancesComparison)
{
	typedef iLQG<state_dim, control_dim>::Policy_t Policy;

	try {

		std::cout << "setting up problem " << std::endl;

		Eigen::Vector2d x_final;
		x_final << 20, 0;

		iLQGSettings ilqg_settings;
		ilqg_settings.epsilon = 0.0;
		ilqg_settings.nThreads = 4;
		ilqg_settings.max_iterations = 50;
		ilqg_settings.recordSmallestEigenvalue = true;
		ilqg_settings.min_cost_improvement = 1e-6;

		shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
		shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
		shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

		// times
		ct::core::Time tf = 3.0;

		// init state
		StateVector<state_dim> x0;  x0.setRandom();

		// construct single-core single subsystem OptCon Problem
		OptConProblem<state_dim, control_dim> optConProblem (tf, x0, nonlinearSystem, costFunction, analyticLinearSystem);

		size_t nSteps = std::round(tf / ilqg_settings.dt);

		std::cout << "initializing ilqg solver" << std::endl;
		iLQG<state_dim, control_dim> ilqg(optConProblem, ilqg_settings);
		iLQGMP<state_dim, control_dim> ilqg_mp(optConProblem, ilqg_settings);

		iLQG<state_dim, control_dim> ilqg_comp(optConProblem, ilqg_settings);
		iLQGMP<state_dim, control_dim> ilqg_mp_comp(optConProblem, ilqg_settings);


		// provide initial controller
		FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
		ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
		iLQG<state_dim, control_dim>::Policy_t initController (u0_ff, u0_fb, ilqg_settings.dt);


		size_t nTests = 2;
		for (size_t i=0; i<nTests; i++)
		{
			if (i==0)
				ilqg_settings.lineSearchSettings.active = false;
			else
				ilqg_settings.lineSearchSettings.active = true;

			ilqg.configure(ilqg_settings);
			ilqg_mp.configure(ilqg_settings);

			ilqg.setInitialGuess(initController);
			ilqg_mp.setInitialGuess(initController);

			ilqg.solve();
			ilqg_mp.solve();


			// make sure this is really the optimal policy and we get a 'false' as return.
			Policy optimalPolicy = ilqg.getSolution();
			Policy optimalPolicy_mp = ilqg_mp.getSolution();

//			ilqg.setInitialGuess(optimalPolicy);
//			ASSERT_FALSE(ilqg.runIteration());
//			ilqg_mp.setInitialGuess(optimalPolicy_mp);
//			ASSERT_FALSE(ilqg_mp.runIteration());

			// now check against the comparison iLQG instances
			ilqg_comp.configure(ilqg_settings);
			ilqg_mp_comp.configure(ilqg_settings);

			ilqg_comp.setInitialGuess(optimalPolicy);
			ilqg_mp_comp.setInitialGuess(optimalPolicy_mp);

			ilqg_comp.solve();
			ilqg_mp_comp.solve();

			Policy optimalPolicy_comp = ilqg_comp.getSolution();
			Policy optimalPolicy_mp_comp = ilqg_mp_comp.getSolution();


			// compare controller sizes
			ASSERT_EQ(optimalPolicy_comp.uff().size(),nSteps);

			ASSERT_EQ(optimalPolicy_comp.uff().size(), optimalPolicy.uff().size());

			// compare controller durations
			ASSERT_EQ(optimalPolicy_comp.getFeedforwardTrajectory().duration(), optimalPolicy.getFeedforwardTrajectory().duration());


			// compare controllers for single core and mp case
			for(size_t i = 0; i<optimalPolicy_comp.uff().size()-1; i++)
			{
				ASSERT_NEAR(optimalPolicy_comp.uff()[i](0), optimalPolicy.uff()[i](0), 1e-3);

				ASSERT_NEAR(optimalPolicy_comp.K()[i].array().abs().maxCoeff(), optimalPolicy.K()[i].array().abs().maxCoeff(), 1e-3);

				ASSERT_NEAR(optimalPolicy_mp_comp.uff()[i](0), optimalPolicy_mp.uff()[i](0), 1e-3);

				ASSERT_NEAR(optimalPolicy_mp_comp.K()[i].array().abs().maxCoeff(), optimalPolicy_mp.K()[i].array().abs().maxCoeff(), 1e-3);
			}

		}

	} catch (std::exception& e)
	{
		std::cout << "caught exception: "<<e.what() <<std::endl;
		FAIL();
	}
}


TEST(ILQCTest, PolicyComparison)
{
	try {

		std::cout << "setting up problem " << std::endl;

		Eigen::Vector2d x_final;
		x_final << 20, 0;

		iLQGSettings ilqg_settings;
		ilqg_settings.epsilon = 0.0;
		ilqg_settings.nThreads = 4;
		ilqg_settings.max_iterations = 50;
		ilqg_settings.recordSmallestEigenvalue = true;
		ilqg_settings.min_cost_improvement = 1e-6;

		shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
		shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
		shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

		// times
		ct::core::Time tf = 3.0;

		// init state
		StateVector<state_dim> x0;  x0.setRandom();

		// construct single-core single subsystem OptCon Problem
		OptConProblem<state_dim, control_dim> optConProblem (tf, x0, nonlinearSystem, costFunction, analyticLinearSystem);

		size_t nSteps = std::round(tf / ilqg_settings.dt);

		std::cout << "initializing ilqg solver" << std::endl;
		iLQG<state_dim, control_dim> ilqg(optConProblem, ilqg_settings);
		iLQGMP<state_dim, control_dim> ilqg_mp(optConProblem, ilqg_settings);


		// provide initial controller
		FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
		ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
		iLQG<state_dim, control_dim>::Policy_t initController (u0_ff, u0_fb, ilqg_settings.dt);

		ilqg.configure(ilqg_settings);
		ilqg.setInitialGuess(initController);

		ilqg.solve();

		size_t nTests = 2;
		for (size_t i=0; i<nTests; i++)
		{
			if (i==0)
				ilqg_settings.lineSearchSettings.active = false;
			else
				ilqg_settings.lineSearchSettings.active = true;

			ilqg.configure(ilqg_settings);
			ilqg_mp.configure(ilqg_settings);

			ilqg.setInitialGuess(initController);
			ilqg_mp.setInitialGuess(initController);

			bool foundBetter = true;
			bool foundBetter_mp = true;
			size_t numIterations = 0;

			while (foundBetter)
			{
				// solve
				foundBetter = ilqg.runIteration();
				foundBetter_mp = ilqg_mp.runIteration();

				numIterations++;

				// we should converge in way less than 20 iterations
				ASSERT_LT(numIterations, 20);
			}


			// test trajectories
			StateTrajectory<state_dim> xRollout = ilqg.getStateTrajectory();
			StateTrajectory<state_dim> xRollout_mp = ilqg_mp.getStateTrajectory();

			// the optimal controller
			std::shared_ptr<iLQG<state_dim, control_dim>::Policy_t> optController (new iLQG<state_dim, control_dim>::Policy_t(ilqg.getSolution()));
			std::shared_ptr<iLQG<state_dim, control_dim>::Policy_t> optController_mp (new iLQG<state_dim, control_dim>::Policy_t(ilqg_mp.getSolution()));

			// two test systems
			std::shared_ptr<ControlledSystem<state_dim, control_dim> > testSystem1 (new Dynamics);
			std::shared_ptr<ControlledSystem<state_dim, control_dim> > testSystem2 (new Dynamics);

			// set the controller
			testSystem1->setController(optController);
			testSystem2->setController(optController_mp);

			// test integrators, the same as in iLQG
			ct::core::IntegratorRK4<state_dim> testIntegrator1 (testSystem1);
			ct::core::IntegratorRK4<state_dim> testIntegrator2 (testSystem2);

			// states
			ct::core::StateVector<state_dim> x_test_1 = x0;
			ct::core::StateVector<state_dim> x_test_2 = x0;

			// do forward integration -- should be the same as in
			testIntegrator1.integrate_n_steps(x_test_1, 0.0, nSteps, ilqg_settings.dt_sim);
			testIntegrator2.integrate_n_steps(x_test_2, 0.0, nSteps, ilqg_settings.dt_sim);

			ASSERT_LT((x_test_1 - xRollout.back()).array().abs().maxCoeff(), 0.3);
			ASSERT_LT((x_test_2 - xRollout_mp.back()).array().abs().maxCoeff(), 0.3);

		}

	} catch (std::exception& e)
	{
		std::cout << "caught exception: "<<e.what() <<std::endl;
		FAIL();
	}
}


} // namespace example
} // namespace optcon
} // namespace ct


/*!
 * This runs the iLQG unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example iLQGCTest.cpp
 */
int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
