/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

//#define DEBUG_PRINT_MPC
//#define MATLAB_LOG_MPC

#pragma once

#include <chrono>
#include <gtest/gtest.h>

#include "mpcTestSettings.h"
#include "../testSystems/LinearOscillator.h"

namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;


/**
 * Test the MPC pre integrator
 */
TEST(MPCTestA, DISABLED_PreIntegratorTest)
{
    typedef tpl::LinearOscillator<double> LinearOscillator;
    typedef tpl::LinearOscillatorLinear<double> LinearOscillatorLinear;

    try
    {
        // desired final state
        Eigen::Matrix<double, state_dim, 1> x_final;
        x_final << 20, 0;

        StateVector<state_dim> x0;
        x0.setRandom();            // init state
        double timeHorizon = 3.0;  // final time


        // set up the Optimal Control Problem
        shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator);
        shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear);
        shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
            tpl::createCostFunctionLinearOscillator<double>(x_final);

        OptConProblem<state_dim, control_dim> optConProblem(nonlinearSystem, costFunction, analyticLinearSystem);
        optConProblem.setInitialState(x0);
        optConProblem.setTimeHorizon(timeHorizon);

        // FIRST ILQR INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS
        NLOptConSettings nloc_settings;
        nloc_settings.dt = 0.001;
        nloc_settings.max_iterations = 100;
        nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
        nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
        nloc_settings.closedLoopShooting = true;
        nloc_settings.integrator = ct::core::IntegrationType::EULER;
        nloc_settings.printSummary = false;

        // number of steps
        size_t K = std::round(timeHorizon / nloc_settings.dt);

        // initial controller
        FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
        ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Ones());
        StateVectorArray<state_dim> x_ref(K + 1, x0);
        ct::core::StateFeedbackController<state_dim, control_dim> initController(x_ref, u0_ff, u0_fb, nloc_settings.dt);

        NLOptConSolver<state_dim, control_dim> initSolver(optConProblem, nloc_settings);
        initSolver.configure(nloc_settings);
        initSolver.setInitialGuess(initController);
        bool boolInitSuccess = initSolver.solve();

        // obtain the 'perfect' init controller from first iLQR solver
        ct::core::StateFeedbackController<state_dim, control_dim> perfectInitController = initSolver.getSolution();
        auto perfectStateTrajectory = initSolver.getStateTrajectory();

        // mpc settings
        ct::optcon::mpc_settings settings_mpc;
        settings_mpc.stateForwardIntegration_ = true;
        settings_mpc.postTruncation_ = false;

        // MPC instance
        MPC<NLOptConSolver<state_dim, control_dim>> mpcSolver(optConProblem, nloc_settings, settings_mpc);

        // initialize it with perfect initial guess
        mpcSolver.setInitialGuess(perfectInitController);

        ct::core::Time t = 0.0;  // init time


        ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;
        ct::core::Time ts_newPolicy;


        // run one mpc cycle
        bool success = mpcSolver.run(x0, t, newPolicy, ts_newPolicy);

        auto mpcStateTrajectory = newPolicy.getReferenceStateTrajectory();


        // after one mpc cycle the solution should only slightly deviate
        ASSERT_EQ(newPolicy.uff().size(), perfectInitController.uff().size());
        ASSERT_EQ(newPolicy.getFeedforwardTrajectory().duration(),
            perfectInitController.getFeedforwardTrajectory().duration());

        for (size_t i = 0; i < mpcStateTrajectory.size(); i++)
        {
            //			std::cout << " mpc traj: " << mpcStateTrajectory[i].transpose();
            //			std::cout << "\t nominal state " << perfectStateTrajectory[i].transpose() << std::endl;
            ASSERT_NEAR(mpcStateTrajectory[i](0), perfectStateTrajectory[i](0), 0.03);  // positions
            ASSERT_NEAR(mpcStateTrajectory[i](1), perfectStateTrajectory[i](1), 0.2);   // velocities
        }

        // test the forward integration scheme with external controller
        std::shared_ptr<ct::core::StateFeedbackController<state_dim, control_dim>> prevController(
            new ct::core::StateFeedbackController<state_dim, control_dim>(newPolicy));
        for (size_t i = 0; i < mpcStateTrajectory.size(); i += 200)
        {
            ct::core::StateVector<state_dim> state = mpcStateTrajectory.front();
            mpcSolver.doPreIntegration(0.0, i * nloc_settings.dt, state, prevController);

            ASSERT_LT(fabs((state(0) - mpcStateTrajectory[i](0))), 0.03);  // position is allowed to vary 3 cm
        }


        // test the forward integration scheme with internal controller
        for (size_t i = 0; i < mpcStateTrajectory.size(); i += 200)
        {
            ct::core::StateVector<state_dim> state = mpcStateTrajectory.front();
            mpcSolver.doPreIntegration(0.0, i * nloc_settings.dt, state);

            ASSERT_LT(
                fabs((state(0) - mpcStateTrajectory[i](0))), 0.03);  // position is allowed to vary a couple of [cm]

            // std::cout << "pre-int state " << state.transpose() << std::endl;
            // std::cout << "nominal state " << x_traj[i].transpose() << std::endl;
        }


    } catch (std::exception& e)
    {
        std::cout << "caught exception: " << e.what() << std::endl;
        FAIL();
    }
}


TEST(MPCTestB, DISABLED_NLOC_MPC_DoublePrecision)
{
    typedef tpl::LinearOscillator<double> LinearOscillator;
    typedef tpl::LinearOscillatorLinear<double> LinearOscillatorLinear;

    for (int solverType = 0; solverType <= 1; solverType++)

        try
        {
            Eigen::Vector2d x_final;
            x_final << 20, 0;

            StateVector<state_dim> x0;
            x0.setRandom();

            ct::core::Time timeHorizon = 3.0;

            // set up the Optimal Control Problem
            shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator);
            shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear);
            shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
                tpl::createCostFunctionLinearOscillator<double>(x_final);

            OptConProblem<state_dim, control_dim> optConProblem(nonlinearSystem, costFunction, analyticLinearSystem);

            optConProblem.setTimeHorizon(timeHorizon);

            optConProblem.setInitialState(x0);


            // FIRST ILQR INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS

            NLOptConSettings nloc_settings;
            nloc_settings.dt = 0.001;
            nloc_settings.K_sim = 1;
            nloc_settings.K_shot = 1;
            nloc_settings.max_iterations = 10;
            nloc_settings.min_cost_improvement = 1e-10;  // strict bounds to reach a solution very close to optimality
            nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
            nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
            nloc_settings.closedLoopShooting = true;
            nloc_settings.integrator = ct::core::IntegrationType::RK4;
            nloc_settings.lineSearchSettings.active = false;
            nloc_settings.nThreads = 1;
            nloc_settings.nThreadsEigen = 1;
            nloc_settings.printSummary = false;
            nloc_settings.debugPrint = false;
            nloc_settings.timeVaryingDiscretization = false;


            if (solverType == 0)
                nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
            else
                nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;


            size_t K = std::round(timeHorizon / nloc_settings.dt);  // number of steps


            // provide initial controller
            FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
            ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
            StateVectorArray<state_dim> x_ref(K + 1, x0);
            ct::core::StateFeedbackController<state_dim, control_dim> initController(
                x_ref, u0_ff, u0_fb, nloc_settings.dt);


            // solve iLQR and obtain perfect init guess
            NLOptConSolver<state_dim, control_dim> initSolver(optConProblem, nloc_settings);

            initSolver.configure(nloc_settings);

            initSolver.setInitialGuess(initController);

            initSolver.solve();

            ct::core::StateFeedbackController<state_dim, control_dim> perfectInitController = initSolver.getSolution();
            ct::core::StateTrajectory<state_dim> perfectStateTrajectory =
                perfectInitController.getReferenceStateTrajectory();


            // settings for the ilqr instance used in MPC
            NLOptConSettings nloc_settings_mpc = nloc_settings;
            nloc_settings_mpc.max_iterations = 1;
            nloc_settings_mpc.lineSearchSettings.active = false;

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
            MPC<NLOptConSolver<state_dim, control_dim>> mpcSolver(optConProblem, nloc_settings_mpc, settings);

            mpcSolver.setInitialGuess(perfectInitController);

            // fake the time -- here the start time
            auto start_time = std::chrono::high_resolution_clock::now();

            // outputs
            std::vector<ct::core::StateTrajectory<state_dim>>
                stateTrajContainer;  // collection of all state trajectories
            ct::core::StateTrajectory<state_dim> tempStateTraj;
            std::vector<double> timeStamps;                            // collection of all policy-start timestamps
            std::vector<ct::core::StateVector<state_dim>> initStates;  // collection of all initial tests


            size_t maxNumRuns = 2000;
            size_t numRuns = 0;

            std::cout << "Starting to run MPC" << std::endl;

            for (size_t i = 0; i < maxNumRuns; i++)
            {
                // we assume to have a perfect initial state (perfect state evolution)
                if (i > 0)
                    x0 = tempStateTraj.front();

                // time which has passed since start of MPC
                auto current_time = std::chrono::high_resolution_clock::now();
                ct::core::Time t =
                    0.000001 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

                // new optimal policy
                ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

                // timestamp of the new optimal policy
                ct::core::Time ts_newPolicy;

                // run one mpc cycle
                bool success = mpcSolver.run(x0, t, newPolicy, ts_newPolicy);

                tempStateTraj = newPolicy.getReferenceStateTrajectory();

                // we save every 20-th trajectory
                if (i % 20 == 0)
                {
                    stateTrajContainer.push_back(tempStateTraj);
                    timeStamps.push_back(ts_newPolicy);
                    initStates.push_back(x0);
                }


                if (mpcSolver.timeHorizonReached() || !success)
                    break;

                numRuns++;
            }


            mpcSolver.printMpcSummary();

            ASSERT_GT(numRuns, 10);  // make sure that MPC runs more than 10 times


            //		Intuition:
            //		The start of every mpc state trajectory must lie on the initial "perfect" state trajectory,
            //		since we have negligible delays here, a close-to-perfect state 'measurement' and no perturbations.
            //		the perfect state trajectory above starts at t=0 and the init time-stamp
            //		of the first MPC solution is the following

            ct::core::Time mpcTimeOffset = timeStamps.front();
            std::cout << "mpc trajectories time offset due to init solve: " << mpcTimeOffset << std::endl;


//		 Reasons why this unit test might fail: too high delays.
//		 - not building in Release Mode ?
//		 - printouts enabled ?


#ifdef MATLAB_LOG_MPC
#ifdef MATLAB
            std::cout << "Saving MPC trajectories to Matlab" << std::endl;

            matlab::MatFile matFile;
            std::string dir = std::string(DATA_DIR_MPC) + "/solution.mat";
            matFile.open(dir);

            for (size_t i = 0; i < timeStamps.size(); i++)
            {
                std::string x_varName = "x_" + std::to_string(i);    // state traj
                std::string t_varName = "t_" + std::to_string(i);    // time traj
                std::string ts_varName = "ts_" + std::to_string(i);  // time stamp
                std::string x0_varName = "x0_" + std::to_string(i);  // initial states
                matFile.put(x_varName, stateTrajContainer[i].getDataArray().toImplementation());
                matFile.put(t_varName, stateTrajContainer[i].getTimeArray().toEigenTrajectory());
                matFile.put(ts_varName, timeStamps[i]);
            }

            matFile.close();
#endif
#endif

            for (size_t i = 0; i < stateTrajContainer.size(); i++)
            {
                ct::core::Time relTime = timeStamps[i] - mpcTimeOffset;
                ct::core::StateVector<state_dim> mpcTrajInitState = stateTrajContainer[i].front();
                ct::core::StateVector<state_dim> refState = stateTrajContainer[0].eval(relTime);

                //			std::cout << "x_ref: " << refState.transpose() << std::endl;
                //			std::cout << "x_mpc: " << mpcTrajInitState.transpose() << std::endl;

                ASSERT_LT(std::fabs((refState - mpcTrajInitState)(0)), 1.0);  // max pos deviation
                ASSERT_LT(std::fabs((refState - mpcTrajInitState)(1)), 1.0);  // max vel deviation
            }


        } catch (std::exception& e)
        {
            std::cout << "caught exception: " << e.what() << std::endl;
            FAIL();
        }
}


/*
 * Note: the single-precision test does not yet include HPIPM. HPIPM single-precision is still under development.
 */
//TEST(MPCTest, iLQRMPC_SinglePrecision)
//{
//	typedef tpl::LinearOscillator<float> LinearOscillator;
//	typedef tpl::LinearOscillatorLinear<float> LinearOscillatorLinear;
//
//	try {
//
//		Eigen::Vector2f x_final; x_final << 20, 0;
//
//		StateVector<state_dim, float> x0; x0.setRandom();
//
//		float timeHorizon = 3.0;
//
//		// set up the Optimal Control Problem
//		shared_ptr<ControlledSystem<state_dim, control_dim, float> > nonlinearSystem(new LinearOscillator());
//		shared_ptr<LinearSystem<state_dim, control_dim, float> > analyticLinearSystem(new LinearOscillatorLinear());
//		shared_ptr<CostFunctionQuadratic<state_dim, control_dim, float> > costFunction = tpl::createCostFunctionLinearOscillator<float>(x_final);
//
//		OptConProblem<state_dim, control_dim, float> optConProblem (nonlinearSystem, costFunction, analyticLinearSystem);
//
//		optConProblem.setTimeHorizon(timeHorizon);
//
//		optConProblem.setInitialState(x0);
//
//
//		// FIRST ILQR INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS
//
//		NLOptConSettings nloc_settings;
//		nloc_settings.dt = 0.001;
//		nloc_settings.K_sim = 1;
//		nloc_settings.K_shot = 1;
//		nloc_settings.max_iterations = 10;
//		nloc_settings.min_cost_improvement = 1e-10;	// strict bounds to reach a solution very close to optimality
//		nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
//		nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
//		nloc_settings.closedLoopShooting = true;
//		nloc_settings.integrator = ct::core::IntegrationType::RK4;
//		nloc_settings.lineSearchSettings.active = false;
//		nloc_settings.nThreads = 3;
//		nloc_settings.nThreadsEigen = 1;
//		nloc_settings.printSummary = false;
//		nloc_settings.debugPrint = false;
//		nloc_settings.timeVaryingDiscretization = false;
//
//
//		size_t K = std::round(timeHorizon / nloc_settings.dt); // number of steps
//
//
//		// provide initial controller
//		FeedbackArray<state_dim, control_dim, float> u0_fb(K, FeedbackMatrix<state_dim, control_dim, float>::Zero());
//		ControlVectorArray<control_dim, float> u0_ff(K, ControlVector<control_dim, float>::Zero());
//		StateVectorArray<state_dim, float> x_ref (K+1, x0);
//		ct::core::StateFeedbackController<state_dim, control_dim, float> initController (x_ref, u0_ff, u0_fb, nloc_settings.dt);
//
//
//		// solve iLQR and obtain perfect init guess
//		NLOptConSolver<state_dim, control_dim, state_dim/2, state_dim/2, float> initSolver (optConProblem, nloc_settings);
//
//		initSolver.configure(nloc_settings);
//
//		initSolver.setInitialGuess(initController);
//
//		initSolver.solve();
//
//		ct::core::StateFeedbackController<state_dim, control_dim, float> perfectInitController = initSolver.getSolution();
//		ct::core::StateTrajectory<state_dim, float> perfectStateTrajectory = perfectInitController.getReferenceStateTrajectory();
//
//
//		// settings for the ilqr instance used in MPC
//		NLOptConSettings nloc_settings_mpc = nloc_settings;
//		nloc_settings_mpc.max_iterations = 5;
//
//		// mpc specific settings
//		ct::optcon::mpc_settings settings;
//		settings.stateForwardIntegration_ = true;
//		settings.postTruncation_ = false;
//		settings.measureDelay_ = true;
//		settings.delayMeasurementMultiplier_ = 1.0;
//		settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
//		settings.coldStart_ = false;
//		settings.additionalDelayUs_ = 0;
//
//		// Create MPC object
//		MPC<NLOptConSolver<state_dim, control_dim, state_dim/2, state_dim/2, float>> mpcSolver (optConProblem, nloc_settings_mpc, settings);
//
//		mpcSolver.setInitialGuess(perfectInitController);
//
//
//		// fake the time -- here the start time
//		auto start_time = std::chrono::high_resolution_clock::now();
//
//		// outputs
//		std::vector<ct::core::StateTrajectory<state_dim, float>> stateTrajContainer;	// collection of all state trajectories
//		ct::core::StateTrajectory<state_dim, float> tempStateTraj;
//		std::vector<float> timeStamps;	// collection of all policy-start timestamps
//		std::vector<ct::core::StateVector<state_dim, float>> initStates; // collection of all initial tests
//
//
//		size_t maxNumRuns = 2000;
//		size_t numRuns = 0;
//
//		std::cout << "Starting to run MPC" << std::endl;
//
//		for(size_t i = 0; i<maxNumRuns; i++)
//		{
//
//			// we assume to have a perfect initial state (perfect state evolution)
//			if(i>0)
//				x0 = tempStateTraj.front();
//
//			// time which has passed since start of MPC
//			auto current_time = std::chrono::high_resolution_clock::now();
//			float t = 0.000001*std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
//
//
//			// new optimal policy
//			ct::core::StateFeedbackController<state_dim, control_dim, float> newPolicy;
//
//			// timestamp of the new optimal policy
//			float ts_newPolicy;
//
//
//			// run one mpc cycle
//			bool success = mpcSolver.run(x0, t, newPolicy, ts_newPolicy);
//
//
//			tempStateTraj = newPolicy.getReferenceStateTrajectory();
//
//			// we save every 20-th trajectory
//			if(i%20 == 0)
//			{
//				stateTrajContainer.push_back(tempStateTraj);
//				timeStamps.push_back(ts_newPolicy);
//				initStates.push_back(x0);
//			}
//
//
//			if(mpcSolver.timeHorizonReached() || !success)
//				break;
//
//			numRuns++;
//		}
//
//
//		mpcSolver.printMpcSummary();
//
//		ASSERT_GT(numRuns, 10); // make sure that MPC runs more than 10 times
//
//
////		 * Intuition:
////		 * The start of every mpc state trajectory must lie on the initial "perfect" state trajectory,
////		 * since we have negligible delays here, a close-to-perfect state 'measurement' and no perturbations.
////		 * the perfect state trajectory above starts at t=0 and the init time-stamp
////		 * of the first MPC solution is the following
//
//		float mpcTimeOffset = timeStamps.front();
//		std::cout << "mpc trajectories time offset due to init solve: " << mpcTimeOffset << std::endl;
//
//		for(size_t i = 0; i< stateTrajContainer.size(); i++)
//		{
//			float relTime = timeStamps[i] - mpcTimeOffset;
//			ct::core::StateVector<state_dim, float> mpcTrajInitState = stateTrajContainer[i].front();
//			ct::core::StateVector<state_dim, float> refState = stateTrajContainer[0].eval(relTime);
//
////			std::cout << "x_ref: " << refState.transpose() << std::endl;
////			std::cout << "x_mpc: " << mpcTrajInitState.transpose() << std::endl;
//
//			ASSERT_LT(std::fabs((refState-mpcTrajInitState)(0)), 1.0);	// max pos deviation
//			ASSERT_LT(std::fabs((refState-mpcTrajInitState)(1)), 1.0);	// max vel deviation
//		}
//	} catch (std::exception& e)
//	{
//		std::cout << "caught exception: "<<e.what() <<std::endl;
//		FAIL();
//	}
//}


}  // namespace example
}  // namespace optcon
}  // namespace ct
