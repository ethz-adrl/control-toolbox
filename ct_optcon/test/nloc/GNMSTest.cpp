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
//#include <gtest/gtest.h>

//#define MATLAB
//#define MATLAB_FULL_LOG

#define DEBUG_PRINT
//#define DEBUG_PRINT_MP
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



/*!
 * variations:
 * single shooting ST/MP  LS/noLS
 * iLQR ST/MP LS/noLS
 * GNMS LS/nlLS, ST/MP
 * Multiple Shooting
 * iLQR-GNMS(M)
 *
 * all with HPIPM or without HPIPM solver
 *
 * todo: all converge in 1st iteration
 * all have same cost after 1 iteration
 *
 */


void testGaussNewtonMethods()
{
	typedef NLOptConSolver<state_dim, control_dim, state_dim /2, state_dim /2> NLOptConSolver;

	Eigen::Vector2d x_final;
	x_final << 20, 0;


	NLOptConSettings nloc_settings;
	nloc_settings.epsilon = 0.0;
	nloc_settings.max_iterations = 1;
	nloc_settings.recordSmallestEigenvalue = false;
	nloc_settings.min_cost_improvement = 1e-6;
	nloc_settings.fixedHessianCorrection = false;
	nloc_settings.dt = 0.001;
	nloc_settings.dt_sim = 0.001;
	nloc_settings.dt_shot = 0.001;
	nloc_settings.integrator = ct::core::IntegrationType::EULER;
	nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
	nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
	nloc_settings.stabilizeAroundPreviousSolution = false;

	// loop through all solver classes
	for(int algClass = 0; algClass < NLOptConSettings::NLOCP_ALGORITHM::NUM_TYPES; algClass++)
	{
		nloc_settings.nlocp_algorithm = static_cast<NLOptConSettings::NLOCP_ALGORITHM>(algClass);

		// change between closed-loop and open-loop
		for(int toggleClosedLoop = 0; toggleClosedLoop<=1; toggleClosedLoop++)
		{
			nloc_settings.closedLoopShooting = (bool) toggleClosedLoop;

			// switch line search on or off
			for(int toggleLS = 0; toggleLS <= 1; toggleLS++)
			{
				nloc_settings.lineSearchSettings.active = (bool)toggleLS;

				for(size_t nThreads = 1; nThreads<5; nThreads=nThreads+3)
				{
					nloc_settings.nThreads = nThreads;

					std::cout << "testing variant with " << nloc_settings.nThreads << " threads, lineSearch " <<toggleLS << " closedLoop " <<toggleClosedLoop << ", algClass " << algClass << std::endl;

					// start test solver ==============================================================================
					shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
					shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
					shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

					// times
					ct::core::Time tf = 3.0;
					size_t nSteps = std::round(tf / nloc_settings.dt);

					// provide initial guess
					StateVector<state_dim> initState; initState.setZero(); initState(1) = 1.0;
					StateVectorArray<state_dim>  x0(nSteps+1, initState);
					ControlVector<control_dim> uff; uff << kStiffness * initState(0);
					ControlVectorArray<control_dim> u0(nSteps, uff);


					FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
					ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());

					NLOptConSolver::Policy_t initController (x0, u0, u0_fb, nloc_settings.dt);

					// construct single-core single subsystem OptCon Problem
					OptConProblem<state_dim, control_dim> optConProblem (tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


					std::cout << "initializing gnms solver" << std::endl;
					NLOptConSolver solver(optConProblem, nloc_settings);

					solver.configure(nloc_settings);
					solver.setInitialGuess(initController);
					solver.runIteration(); // must be converged after 1 iteration

					// test trajectories
					StateTrajectory<state_dim> xRollout = solver.getStateTrajectory();
					ControlTrajectory<control_dim> uRollout = solver.getControlTrajectory();

					std::cout<<"x final GNMS: " << xRollout.back().transpose() << std::endl;
					std::cout<<"u final GNMS: " << uRollout.back().transpose() << std::endl;

					// end test solver ===========================================================================

				} // toggle multi-threading / single-threading
			} // toggle line-search
		} // toggle closed-loop
	} // toggle solver class
} // end test



void singleCore()
{
	std::cout << "setting up problem " << std::endl;

	typedef NLOptConSolver<state_dim, control_dim, state_dim /2, state_dim /2> NLOptConSolver;

	Eigen::Vector2d x_final;
	x_final << 20, 0;

	NLOptConSettings gnms_settings;
	gnms_settings.nThreads = 1;
	gnms_settings.epsilon = 0.0;
	gnms_settings.max_iterations = 1;
	gnms_settings.recordSmallestEigenvalue = false;
	gnms_settings.min_cost_improvement = 1e-6;
	gnms_settings.fixedHessianCorrection = false;
	gnms_settings.dt = 0.001;
	gnms_settings.dt_sim = 0.001;
	gnms_settings.dt_shot = 0.001;
	gnms_settings.integrator = ct::core::IntegrationType::EULER;
	gnms_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
	gnms_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
	gnms_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
	gnms_settings.closedLoopShooting = false;
	gnms_settings.lineSearchSettings.active = false;
	gnms_settings.loggingPrefix = "GNMS";

	NLOptConSettings ilqr_settings = gnms_settings;
	ilqr_settings.closedLoopShooting = true;
	ilqr_settings.stabilizeAroundPreviousSolution = false;
	ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
	ilqr_settings.loggingPrefix = "ILQR";


	shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
	shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
	shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

	// times
	ct::core::Time tf = 3.0;
	size_t nSteps = std::round(tf / gnms_settings.dt);

	// provide initial guess
	StateVector<state_dim> initState; initState.setZero(); initState(1) = 1.0;
	StateVectorArray<state_dim>  x0(nSteps+1, initState);
	ControlVector<control_dim> uff; uff << kStiffness * initState(0);
	ControlVectorArray<control_dim> u0(nSteps, uff);
	for (size_t i=0; i<nSteps+1; i++)
	{
		//			x0 [i] = x_final*double(i)/double(nSteps);
	}

	FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
	ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());

	NLOptConSolver::Policy_t initController (x0, u0, u0_fb, gnms_settings.dt);

	// construct single-core single subsystem OptCon Problem
	OptConProblem<state_dim, control_dim> optConProblem (tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


	std::cout << "initializing gnms solver" << std::endl;
	NLOptConSolver gnms(optConProblem, gnms_settings);
	NLOptConSolver ilqr(optConProblem, ilqr_settings);


	gnms.configure(gnms_settings);
	gnms.setInitialGuess(initController);

	ilqr.configure(ilqr_settings);
	ilqr.setInitialGuess(initController);

	std::cout << "running gnms solver" << std::endl;

	bool foundBetter = true;
	size_t numIterations = 0;

	while (numIterations<3)
	{
		foundBetter = gnms.runIteration();

		// test trajectories
		StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
		ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

		numIterations++;

		std::cout<<"x final GNMS: " << xRollout.back().transpose() << std::endl;
		std::cout<<"u final GNMS: " << uRollout.back().transpose() << std::endl;
	}

	std::cout << "running ilqr solver" << std::endl;

	numIterations = 0;
	foundBetter = true;
	while (numIterations<3)
	{
		foundBetter = ilqr.runIteration();

		// test trajectories
		StateTrajectory<state_dim> xRollout = ilqr.getStateTrajectory();
		ControlTrajectory<control_dim> uRollout = ilqr.getControlTrajectory();

		numIterations++;

		std::cout<<"x final iLQR: " << xRollout.back().transpose() << std::endl;
		std::cout<<"u final iLQR: " << uRollout.back().transpose() << std::endl;
	}
}


void multiCore()
{
	std::cout << "setting up problem " << std::endl;

	typedef NLOptConSolver<state_dim, control_dim, state_dim /2, state_dim /2> NLOptConSolver;

	Eigen::Vector2d x_final;
	x_final << 20, 0;

	NLOptConSettings gnms_settings;
	gnms_settings.nThreads = 4;
	gnms_settings.nThreadsEigen = 4;
	gnms_settings.epsilon = 0.0;
	gnms_settings.max_iterations = 2;
	gnms_settings.recordSmallestEigenvalue = false;
	gnms_settings.min_cost_improvement = 1e-6;
	gnms_settings.fixedHessianCorrection = false;
	gnms_settings.dt = 0.001;
	gnms_settings.dt_sim = 0.001;
	gnms_settings.dt_shot = 0.023;
	gnms_settings.integrator = ct::core::IntegrationType::EULER;
	gnms_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
	gnms_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
	gnms_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
	gnms_settings.closedLoopShooting = false;
	gnms_settings.loggingPrefix = "GNMS";
	gnms_settings.lineSearchSettings.active = false;

	NLOptConSettings ilqr_settings = gnms_settings;
	ilqr_settings.closedLoopShooting = true;
	ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
	ilqr_settings.loggingPrefix = "ILQR";


	shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
	shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
	shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

	// times
	ct::core::Time tf = 3.0;
	size_t nSteps = std::round(tf / gnms_settings.dt);

	// provide initial guess
	ControlVectorArray<control_dim> u0(nSteps, ControlVector<control_dim>::Zero());
	StateVector<state_dim> initState; initState.setZero(); initState(1) = 1.0;
	StateVectorArray<state_dim>  x0(nSteps+1, initState);
	for (size_t i=0; i<nSteps+1; i++)
	{
		//			x0 [i] = x_final*double(i)/double(nSteps);
	}

	FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
	ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());

	NLOptConSolver::Policy_t initController (x0, u0, u0_fb, gnms_settings.dt);

	// construct single-core single subsystem OptCon Problem
	OptConProblem<state_dim, control_dim> optConProblem (tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


	std::cout << "initializing gnms solver" << std::endl;
	NLOptConSolver gnms(optConProblem, gnms_settings);
	NLOptConSolver ilqr(optConProblem, ilqr_settings);


	gnms.configure(gnms_settings);
	gnms.setInitialGuess(initController);

	ilqr.configure(ilqr_settings);
	ilqr.setInitialGuess(initController);

	std::cout << "running gnms solver" << std::endl;

	bool foundBetter = true;
	size_t numIterations = 0;

	while (numIterations<3)
	{
		foundBetter = gnms.runIteration();

		// test trajectories
		StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
		ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

		numIterations++;

		std::cout<<"x final GNMS: " << xRollout.back().transpose() << std::endl;
		std::cout<<"u final GNMS: " << uRollout.back().transpose() << std::endl;
	}

	std::cout << "running ilqr solver" << std::endl;

	numIterations = 0;
	foundBetter = true;
	while (numIterations<3)
	{
		foundBetter = ilqr.runIteration();

		// test trajectories
		StateTrajectory<state_dim> xRollout = ilqr.getStateTrajectory();
		ControlTrajectory<control_dim> uRollout = ilqr.getControlTrajectory();

		numIterations++;

		std::cout<<"x final iLQR: " << xRollout.back().transpose() << std::endl;
		std::cout<<"u final iLQR: " << uRollout.back().transpose() << std::endl;
	}
}



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
	for(size_t i = 0; i<100; i++)
		ct::optcon::example::testGaussNewtonMethods();

//	ct::optcon::example::singleCore();
//	ct::optcon::example::multiCore();

	return 1;
}
