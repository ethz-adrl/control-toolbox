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

#include <chrono>

// Bring in gtest
#include <gtest/gtest.h>

//#define DEBUG_PRINT
//#define DEBUG_PRINT_LINESEARCH
//#define MATLAB_LOG_MPC

#include <ct/optcon/optcon.h>

#include <ct/optcon/mpc/MPC.h>

#include <string>

#include "mpcTestSettings.h"


#include <ct/optcon/matlab.hpp>

namespace ct{
namespace optcon{
namespace example{

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;

const size_t state_dim = 2; // position, velocity
const size_t control_dim = 1; // force

const double kStiffness = 10;

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

std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > createCostFunction(Eigen::Vector2d& x_final) {
	Eigen::Matrix2d Q;
	Q << 0, 0, 0, 0.1;

	Eigen::Matrix<double, 1, 1> R;
	R << 0.001;

	Eigen::Vector2d x_nominal = Eigen::Vector2d::Zero();
	Eigen::Matrix<double, 1, 1> u_nominal = Eigen::Matrix<double, 1, 1>::Zero();

	Eigen::Matrix2d Q_final;
	Q_final << 1000, 0, 0, 1000;

	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > quadraticCostFunction(
			new CostFunctionQuadraticSimple<state_dim, control_dim>(
					Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}


/**
 * Test if the MPC - pre integrator gives reasonable results.
 */
TEST(MPCTest, PreIntegratorTest)
{
	try {

		// desired final state
		Eigen::Vector2d x_final; x_final << 20, 0;

		StateVector<state_dim> x0; x0.setRandom();	// init state
		ct::core::Time timeHorizon = 3.0;	// final time


		// set up the Optimal Control Problem
		shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
		shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
		shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

		OptConProblem<state_dim, control_dim> optConProblem (nonlinearSystem, costFunction, analyticLinearSystem);
		optConProblem.setInitialState(x0);
		optConProblem.setTimeHorizon(timeHorizon);

		// FIRST ILQG INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS
		iLQGSettings ilqg_settings;
		ilqg_settings.dt = 0.001;
		ilqg_settings.dt_sim = 0.001;
		ilqg_settings.max_iterations = 100;

		// number of steps
		size_t K = std::round(timeHorizon / ilqg_settings.dt);

		// initial controller
		FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
		ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Ones());
		ct::core::StateFeedbackController<state_dim, control_dim> initController (u0_ff, u0_fb, ilqg_settings.dt);

		iLQG<state_dim, control_dim> iLQG_init (optConProblem, ilqg_settings);
		iLQG_init.configure(ilqg_settings);
		iLQG_init.setInitialGuess(initController);
		iLQG_init.solve();

		// obtain the 'perfect' init controller from first iLQG solver
		ct::core::StateFeedbackController<state_dim, control_dim> perfectInitController = iLQG_init.getSolution();
		auto perfectStateTrajectory = iLQG_init.getStateTrajectory();


		// settings for SECOND ILQG INSTANCE which will run in MPC
		iLQGSettings ilqg_settings_mpc;
		ilqg_settings_mpc.dt = 0.001;
		ilqg_settings_mpc.dt_sim = 0.001;
		ilqg_settings_mpc.max_iterations = 10;

		ct::optcon::mpc_settings settings_mpc;
		settings_mpc.stateForwardIntegration_ = true;
		settings_mpc.postTruncation_ = false;


		// MPC instance
		MPC<iLQG<state_dim, control_dim>> ilqg_mpc (optConProblem, ilqg_settings_mpc, settings_mpc);

		// initialize it with perfect initial guess
		ilqg_mpc.setInitialGuess(perfectInitController);


		ct::core::Time t = 0.0;	// init time


		ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;
		ct::core::Time ts_newPolicy;


		// run one mpc cycle
		bool success = ilqg_mpc.run(x0, t, newPolicy, ts_newPolicy);

		auto mpcStateTrajectory = ilqg_mpc.getStateTrajectory();


		// after one mpc cycle the solution should only slightly deviate
		ASSERT_EQ(newPolicy.uff().size(), perfectInitController.uff().size());
		ASSERT_EQ(newPolicy.getFeedforwardTrajectory().duration(), perfectInitController.getFeedforwardTrajectory().duration());

		for(size_t i = 0; i<mpcStateTrajectory.size(); i++)
		{
				ASSERT_NEAR(mpcStateTrajectory[i](0), perfectStateTrajectory[i](0), 0.03);	// positions
				ASSERT_NEAR(mpcStateTrajectory[i](1), perfectStateTrajectory[i](1), 0.2);	// velocities
		}


		// test the forward integration scheme with external controller
		std::shared_ptr<ct::core::StateFeedbackController<state_dim, control_dim>> prevController (new ct::core::StateFeedbackController<state_dim, control_dim>(newPolicy));
		for(size_t i = 0; i<mpcStateTrajectory.size(); i++)
		{
			ct::core::StateVector<state_dim> state = mpcStateTrajectory.front();
			ilqg_mpc.doPreIntegration(0.0, i*ilqg_settings_mpc.dt, state, prevController);

			ASSERT_LT(fabs((state(0)- mpcStateTrajectory[i](0))), 0.03); 	// position is allowed to vary 3 cm

//			std::cout << "pre-int state " << state.transpose() << std::endl;
//			std::cout << "nominal state " << x_traj[i].transpose() << std::endl;
		}



		// test the forward integration scheme with internal controller
		for(size_t i = 0; i<mpcStateTrajectory.size(); i++)
		{
			ct::core::StateVector<state_dim> state = mpcStateTrajectory.front();
			ilqg_mpc.doPreIntegration(0.0, i*ilqg_settings_mpc.dt, state);

			ASSERT_LT(fabs((state(0)- mpcStateTrajectory[i](0))), 0.03); 	// position is allowed to vary 1 cm

//			std::cout << "pre-int state " << state.transpose() << std::endl;
//			std::cout << "nominal state " << x_traj[i].transpose() << std::endl;
		}


	} catch (std::exception& e)
	{
		std::cout << "caught exception: "<<e.what() <<std::endl;
		FAIL();
	}
}




TEST(MPCTest, iLQGMPC)
{
	try {

		Eigen::Vector2d x_final; x_final << 20, 0;

		StateVector<state_dim> x0; x0.setRandom();

		ct::core::Time timeHorizon = 3.0;

		// set up the Optimal Control Problem
		shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
		shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
		shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

		OptConProblem<state_dim, control_dim> optConProblem (nonlinearSystem, costFunction, analyticLinearSystem);

		optConProblem.setTimeHorizon(timeHorizon);

		optConProblem.setInitialState(x0);


		// FIRST ILQG INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS

		iLQGSettings ilqg_settings;
		ilqg_settings.dt = 0.001;
		ilqg_settings.dt_sim = 0.001;
		ilqg_settings.max_iterations = 10000000;
		ilqg_settings.min_cost_improvement = 0.0;	// strict bounds to reach a solution very close to optimality

		size_t K = std::round(timeHorizon / ilqg_settings.dt); // number of steps


		// provide initial controller
		FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());

		ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());

		ct::core::StateFeedbackController<state_dim, control_dim> initController (u0_ff, u0_fb, ilqg_settings.dt);


		// solve iLQG and obtain perfect init guess
		iLQG<state_dim, control_dim> iLQG_init (optConProblem, ilqg_settings);

		iLQG_init.configure(ilqg_settings);

		iLQG_init.setInitialGuess(initController);

		iLQG_init.solve();

		ct::core::StateFeedbackController<state_dim, control_dim> perfectInitController = iLQG_init.getSolution();
		ct::core::StateTrajectory<state_dim> perfectStateTrajectory = iLQG_init.getStateTrajectory();


		// settings for the ilqg instance used in MPC
		iLQGSettings ilqg_settings_mpc;
		ilqg_settings_mpc.dt = 0.001;
		ilqg_settings_mpc.dt_sim = 0.001;
		ilqg_settings_mpc.max_iterations = 5;


		// mpc specific settings
		ct::optcon::mpc_settings settings;
		settings.stateForwardIntegration_ = true;
		settings.postTruncation_ = true;
		settings.measureDelay_ = true;
		settings.delayMeasurementMultiplier_ = 1.0;
		settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
		settings.coldStart_ = false;
		settings.additionalDelayUs_ = 0;


		// Create MPC object
		MPC<iLQG<state_dim, control_dim>> ilqg_mpc (optConProblem, ilqg_settings_mpc, settings);

		ilqg_mpc.setInitialGuess(perfectInitController);


		// fake the time -- here the start time
		auto start_time = std::chrono::high_resolution_clock::now();

		// outputs
		std::vector<ct::core::StateTrajectory<state_dim>> stateTrajContainer;	// collection of all state trajectories
		ct::core::StateTrajectory<state_dim> tempStateTraj;
		std::vector<double> timeStamps;	// collection of all policy-start timestamps
		std::vector<ct::core::StateVector<state_dim>> initStates; // collection of all initial tests


		size_t maxNumRuns = 2000;
		size_t numRuns = 0;

		std::cout << "Starting to run MPC" << std::endl;

		for(size_t i = 0; i<maxNumRuns; i++)
		{

			// we assume to have a perfect initial state (perfect state evolution)
			if(i>0)
				x0 = tempStateTraj.front();

			// time which has passed since start of MPC
			auto current_time = std::chrono::high_resolution_clock::now();
			ct::core::Time t = 0.000001*std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();


			// new optimal policy
			ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

			// timestamp of the new optimal policy
			ct::core::Time ts_newPolicy;


			// run one mpc cycle
			bool success = ilqg_mpc.run(x0, t, newPolicy, ts_newPolicy);


			tempStateTraj = ilqg_mpc.getStateTrajectory();

			// we save every 20-th trajectory
			if(i%20 == 0)
			{
				stateTrajContainer.push_back(tempStateTraj);
				timeStamps.push_back(ts_newPolicy);
				initStates.push_back(x0);
			}


			if(ilqg_mpc.timeHorizonReached() | !success)
				break;

			numRuns++;
		}


		ilqg_mpc.printMpcSummary();

		ASSERT_GT(numRuns, 10); // make sure that MPC runs more than 10 times


		/*
		 * Intuition:
		 * The start of every mpc state trajectory must lie on the initial "perfect" state trajectory,
		 * since we have negligible delays here, a close-to-perfect state 'measurement' and no perturbations.
		 * the perfect state trajectory above starts at t=0 and the init time-stamp
		 * of the first MPC solution is the following
		 * */
		ct::core::Time mpcTimeOffset = timeStamps.front();
		std::cout << "mpc trajectories time offset due to init solve: " << mpcTimeOffset << std::endl;

		for(size_t i = 0; i< stateTrajContainer.size(); i++)
		{
			ct::core::Time relTime = timeStamps[i] - mpcTimeOffset;
			ct::core::StateVector<state_dim> mpcTrajInitState = stateTrajContainer[i].front();
			ct::core::StateVector<state_dim> refState = stateTrajContainer[0].eval(relTime);

//			std::cout << "x_ref: " << refState.transpose() << std::endl;
//			std::cout << "x_mpc: " << mpcTrajInitState.transpose() << std::endl;

			ASSERT_LT(std::fabs((refState-mpcTrajInitState)(0)), 0.3);	// max pos deviation
			ASSERT_LT(std::fabs((refState-mpcTrajInitState)(1)), 0.3);	// max vel deviation
		}


		/**
		 * Reasons why this unit test might fail: too high delays.
		 * - not building in Release Mode ?
		 * - printouts enabled ?
		 */

#ifdef MATLAB_LOG_MPC
#ifdef MATLAB
		std::cout << "Saving MPC trajectories to Matlab" << std::endl;

		matlab::MatFile matFile;
		std::string dir = std::string(DATA_DIR_MPC) + "/solution.mat";
		matFile.open(dir);

		for(size_t i = 0; i<timeStamps.size(); i++)
		{
			std::string x_varName = "x_"+std::to_string(i);		// state traj
			std::string t_varName = "t_"+std::to_string(i);		// time traj
			std::string ts_varName = "ts_"+std::to_string(i);	// time stamp
			std::string x0_varName = "x0_"+std::to_string(i);	// initial states
			matFile.put(x_varName, stateTrajContainer[i].getDataArray().toImplementation());
			matFile.put(t_varName, stateTrajContainer[i].getTimeArray().toEigenTrajectory());
			matFile.put(ts_varName, timeStamps[i]);
		}

		matFile.close();
#endif
#endif


	} catch (std::exception& e)
	{
		std::cout << "caught exception: "<<e.what() <<std::endl;
		FAIL();
	}
}

} // namespace example
} // namespace optcon
} // namespace ct



int main(int argc, char **argv)
{
	using namespace ct::optcon::example;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
