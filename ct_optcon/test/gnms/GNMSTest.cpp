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
#include <fenv.h>

// Bring in gtest
//#include <gtest/gtest.h>

//#define MATLAB
//#define MATLAB_FULL_LOG

#define DEBUG_PRINT
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

//! Dynamics class for the GNMS unit test
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

//! Linear system class for the GNMS unit test
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

//! Create a cost function for the GNMS unit test
std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > createCostFunction(Eigen::Vector2d& x_final)
{
	Eigen::Matrix2d Q;
	Q << 0, 0, 0, 0;

	Eigen::Matrix<double, 1, 1> R;
	R << 1.0;

	Eigen::Vector2d x_nominal = Eigen::Vector2d::Zero();
	Eigen::Matrix<double, 1, 1> u_nominal = Eigen::Matrix<double, 1, 1>::Zero();

	Eigen::Matrix2d Q_final;
	Q_final << 1000, 0, 0, 10;

	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > quadraticCostFunction(
			new CostFunctionQuadraticSimple<state_dim, control_dim>(
					Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}



void singleCore()
{
		std::cout << "setting up problem " << std::endl;

		Eigen::Vector2d x_final;
		x_final << 20, 0;

		NLOptConSettings gnms_settings;
		gnms_settings.nThreads = 1;
		gnms_settings.epsilon = 0.0;
		gnms_settings.max_iterations = 2;
		gnms_settings.recordSmallestEigenvalue = false;
		gnms_settings.min_cost_improvement = 1e-6;
		gnms_settings.fixedHessianCorrection = false;
		gnms_settings.dt = 0.001;
		gnms_settings.dt_sim = 0.001;
		gnms_settings.integrator = NLOptConSettings::EULER;
		gnms_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
		gnms_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
		gnms_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
		gnms_settings.closedLoopShooting = false;

		NLOptConSettings ilqg_settings = gnms_settings;
		ilqg_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;

		shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
		shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
		shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

		// times
		ct::core::Time tf = 3.0;
		size_t nSteps = std::round(tf / gnms_settings.dt);

		// provide initial guess
		ControlVectorArray<control_dim> u0(nSteps, ControlVector<control_dim>::Zero());
		StateVectorArray<state_dim>  x0(nSteps+1, StateVector<state_dim>::Zero());
		for (size_t i=0; i<nSteps+1; i++)
		{
//			x0 [i] = x_final*double(i)/double(nSteps);
		}

		FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
		ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());

		NLOptConSolver<state_dim, control_dim>::Policy_t initController (x0, u0, u0_fb, gnms_settings.dt);

		// construct single-core single subsystem OptCon Problem
		OptConProblem<state_dim, control_dim> optConProblem (tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


		std::cout << "initializing gnms solver" << std::endl;
		NLOptConSolver<state_dim, control_dim> gnms(optConProblem, gnms_settings);
		NLOptConSolver<state_dim, control_dim> ilqg(optConProblem, ilqg_settings);


		gnms.configure(gnms_settings);
		gnms.setInitialGuess(initController);

		ilqg.configure(ilqg_settings);
		ilqg.setInitialGuess(initController);

		std::cout << "running gnms solver" << std::endl;

		bool foundBetter = true;
		size_t numIterations = 0;

		while (foundBetter)
		{
			foundBetter = gnms.runIteration();

			// test trajectories
			StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
			ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

			numIterations++;

			std::cout<<"x final GNMS: " << xRollout.back().transpose() << std::endl;
			std::cout<<"u final GNMS: " << uRollout.back().transpose() << std::endl;
		}

		std::cout << "running ilqg solver" << std::endl;

		numIterations = 0;
		foundBetter = true;
		while (foundBetter)
		{
			foundBetter = ilqg.runIteration();

			// test trajectories
			StateTrajectory<state_dim> xRollout = ilqg.getStateTrajectory();
			ControlTrajectory<control_dim> uRollout = ilqg.getControlTrajectory();

			numIterations++;

			std::cout<<"x final iLQG: " << xRollout.back().transpose() << std::endl;
			std::cout<<"u final iLQG: " << uRollout.back().transpose() << std::endl;
		}
}

/*


TEST(GNMSTest, PolicyComparison)
{
	try {

		std::cout << "setting up problem " << std::endl;

		Eigen::Vector2d x_final;
		x_final << 20, 0;

		GNMSSettings ilqg_settings;
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
		GNMS<state_dim, control_dim> ilqg(optConProblem, ilqg_settings);
		GNMSMP<state_dim, control_dim> ilqg_mp(optConProblem, ilqg_settings);


		// provide initial controller
		FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
		ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
		GNMS<state_dim, control_dim>::Policy_t initController (u0_ff, u0_fb, ilqg_settings.dt);

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
			std::shared_ptr<GNMS<state_dim, control_dim>::Policy_t> optController (new GNMS<state_dim, control_dim>::Policy_t(ilqg.getSolution()));
			std::shared_ptr<GNMS<state_dim, control_dim>::Policy_t> optController_mp (new GNMS<state_dim, control_dim>::Policy_t(ilqg_mp.getSolution()));

			// two test systems
			std::shared_ptr<ControlledSystem<state_dim, control_dim> > testSystem1 (new Dynamics);
			std::shared_ptr<ControlledSystem<state_dim, control_dim> > testSystem2 (new Dynamics);

			// set the controller
			testSystem1->setController(optController);
			testSystem2->setController(optController_mp);

			// test integrators, the same as in GNMS
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
*/


} // namespace example
} // namespace optcon
} // namespace ct


/*!
 * This runs the GNMS unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example GNMSCTest.cpp
 */
int main(int argc, char **argv)
{
	feenableexcept(FE_INVALID | FE_OVERFLOW);
	ct::optcon::example::singleCore();

	return 1;
}
