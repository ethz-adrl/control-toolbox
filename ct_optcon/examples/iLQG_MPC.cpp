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

#include <ct/optcon/optcon.h>
#include "exampleDir.h"


using namespace ct::core;
using namespace ct::optcon;


/*!
 * This example shows how to use the MPC class. Here, we apply iLQG-MPC to a simple second order system.
 *
 * \example iLQG_MPC.cpp
 */
int main(int argc, char **argv)
{

	const size_t state_dim 		= 2; 	// position, velocity
	const size_t control_dim 	= 1; 	// force


	// STEP 1: set up the Optimal Control Problem

	// create a instance of the oscillator dynamics for the optimal control problem
	double w_n(0.1);
	std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim> > oscillatorDynamics_d(
			new ct::core::SecondOrderSystem(w_n, 5.0));

	// create a System Linearizer
	std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(
			new ct::core::SystemLinearizer<state_dim, control_dim> (oscillatorDynamics_d));

	// load the cost weighting matrices
	std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost (new ct::optcon::TermQuadratic<state_dim, control_dim>());
	std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost (new ct::optcon::TermQuadratic<state_dim, control_dim>());
	intermediateCost->loadConfigFile(ct::optcon::exampleDir+"/mpcCost.info", "intermediateCost", true);
	finalCost->loadConfigFile(ct::optcon::exampleDir+"/mpcCost.info", "finalCost", true);

	// create a cost function
	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction (new CostFunctionAnalytical<state_dim, control_dim>());
	costFunction->addIntermediateTerm(intermediateCost);
	costFunction->addFinalTerm(finalCost);


	// we choose a random initial state
	StateVector<state_dim> x0;
	x0.setRandom();

	// final time in [sec]
	ct::core::Time timeHorizon = 3.0;

	// set up and initialize optimal control problem
	OptConProblem<state_dim, control_dim> optConProblem (oscillatorDynamics_d, costFunction, adLinearizer);
	optConProblem.setInitialState(x0);
	optConProblem.setTimeHorizon(timeHorizon);



	// STEP 2: solve the optimal control problem using iLQG

	iLQGSettings ilqg_settings;
	ilqg_settings.dt = 0.001;
	ilqg_settings.dt_sim = 0.001;
	ilqg_settings.lineSearchSettings.active = false;


	size_t K = std::round(timeHorizon / ilqg_settings.dt); // number of steps

	// provide trivial initial controller to iLQG
	FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
	ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
	ct::core::StateFeedbackController<state_dim, control_dim> initController (u0_ff, u0_fb, ilqg_settings.dt);

	// solve iLQG
	iLQG<state_dim, control_dim> iLQG_init (optConProblem, ilqg_settings);

	iLQG_init.configure(ilqg_settings);

	iLQG_init.setInitialGuess(initController);

	iLQG_init.solve();

	// obtain the optimal controller, which we will use to initialize MPC
	ct::core::StateFeedbackController<state_dim, control_dim> perfectInitController = iLQG_init.getSolution();
	ct::core::StateTrajectory<state_dim> perfectStateTrajectory = iLQG_init.getStateTrajectory();



	// STEP 3: set up MPC

	// settings for the ilqg instance used in MPC
	iLQGSettings ilqg_settings_mpc;
	ilqg_settings_mpc.dt = 0.001;
	ilqg_settings_mpc.dt_sim = 0.001;
	ilqg_settings_mpc.max_iterations = 5; // in MPC-mode, it ususally makes sense to limit the overall number of iLQG iterations

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



	// STEP 4: running MPC
	// MPC needs to receive time information from your control system. Here, "simulate" it using std::chrono
	auto start_time = std::chrono::high_resolution_clock::now();


	// outputs
	ct::core::StateTrajectory<state_dim> stateTraj;

	size_t maxNumRuns = 2000;

	std::cout << "Starting to run MPC" << std::endl;

	for(size_t i = 0; i<maxNumRuns; i++)
	{
		// let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
		if(i>0)
			x0 = stateTraj.front() + 0.1*StateVector<state_dim>::Random();

		// time which has passed since start of MPC
		auto current_time = std::chrono::high_resolution_clock::now();
		ct::core::Time t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

		// new optimal policy
		ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

		// timestamp of the new optimal policy
		ct::core::Time ts_newPolicy;

		// run one mpc cycle
		bool success = ilqg_mpc.run(x0, t, newPolicy, ts_newPolicy);

		// retrieve the currently optimal state trajectory
		stateTraj = ilqg_mpc.getStateTrajectory();

		// we break the loop in case the time horizon is reached or solve() failed
		if(ilqg_mpc.timeHorizonReached() | !success)
			break;
	}


	ilqg_mpc.printMpcSummary();

}
