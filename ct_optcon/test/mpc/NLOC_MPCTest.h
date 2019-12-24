/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
 * Test the MPC forward integrator
 */
TEST(MPCTestA, ForwardIntegratorTest)
{
    typedef tpl::LinearOscillator<double> LinearOscillator;
    typedef tpl::LinearOscillatorLinear<double> LinearOscillatorLinear;

    try
    {
        // desired final state
        Eigen::Matrix<double, state_dim, 1> x_final;
        x_final << 20, 0;

        StateVector<state_dim> x0;
        x0.setRandom();            // initial state
        double timeHorizon = 3.0;  // final time

        // set up the Optimal Control Problem
        shared_ptr<ControlledSystem<state_dim, control_dim>> system(new LinearOscillator);
        shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear);
        shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
            tpl::createCostFunctionLinearOscillator<double>(x_final);

        ContinuousOptConProblem<state_dim, control_dim> optConProblem(system, costFunction, analyticLinearSystem);
        optConProblem.setInitialState(x0);
        optConProblem.setTimeHorizon(timeHorizon);

        // FIRST ILQR INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS
        NLOptConSettings nloc_settings;
        nloc_settings.dt = 0.001;
        nloc_settings.K_sim = 1;  //! required for this test
        nloc_settings.max_iterations = 100;
        nloc_settings.integrator = ct::core::IntegrationType::EULER;
        nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
        nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
        nloc_settings.printSummary = false;

        // number of steps
        size_t K = nloc_settings.computeK(timeHorizon);

        // initial controller
        FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
        ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Ones());
        StateVectorArray<state_dim> x_ref(K + 1, x0);
        ct::core::StateFeedbackController<state_dim, control_dim> initController(x_ref, u0_ff, u0_fb, nloc_settings.dt);

        NLOptConSolver<state_dim, control_dim> initSolver(optConProblem, nloc_settings);
        initSolver.configure(nloc_settings);
        initSolver.setInitialGuess(initController);
        initSolver.solve();

        // obtain the 'perfect' init controller from first iLQR solver
        ct::core::StateFeedbackController<state_dim, control_dim> perfectInitController = initSolver.getSolution();
        auto perfectStateTrajectory = initSolver.getStateTrajectory();

        // mpc settings
        ct::optcon::mpc_settings settings_mpc;
        settings_mpc.stateForwardIntegration_ = true;
        settings_mpc.stateForwardIntegratorType_ = nloc_settings.integrator;
        settings_mpc.stateForwardIntegration_dt_ = nloc_settings.dt;
        settings_mpc.postTruncation_ = false;

        // MPC instance
        MPC<NLOptConSolver<state_dim, control_dim>> mpcSolver(optConProblem, nloc_settings, settings_mpc);

        // initialize it with perfect initial guess
        mpcSolver.setInitialGuess(perfectInitController);

        ct::core::Time t = 0.0;  // init time


        ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;
        ct::core::Time ts_newPolicy;


        /*!
         * Run the first MPC cycle, in which the pre-integrator should not be active at all.
       	 * after one mpc cycle the solution must still be the same (time horizon unchanged, state unchanged)
         */
        mpcSolver.prepareIteration(t);
        mpcSolver.finishIteration(x0, t, newPolicy, ts_newPolicy);
        auto mpcStateTrajectory = newPolicy.getReferenceStateTrajectory();

        ASSERT_EQ(newPolicy.uff().size(), perfectInitController.uff().size());
        ASSERT_EQ(newPolicy.getFeedforwardTrajectory().duration(),
            perfectInitController.getFeedforwardTrajectory().duration());

        for (size_t i = 0; i < mpcStateTrajectory.size(); i++)
        {
            ASSERT_NEAR(mpcStateTrajectory[i](0), perfectStateTrajectory[i](0), 1e-6);  // positions
            ASSERT_NEAR(mpcStateTrajectory[i](1), perfectStateTrajectory[i](1), 1e-6);  // velocities
        }


        /*!
         *  Test the forward integration scheme employing a 'custom controller'.
         *  In this test,
         *  - we select the custom controller to be equal to the previously optimized policy
         *  - we integrate systematically across shifting time intervals
         *  - the integrated solutions need to match the original trajectories with numerical accuracy.
         *
         *  \warning This test is only reasonnable for dt_sim = 1, and is generally problematic
         *  since the solver's forward integration is implemented step-wise.
         */
        std::shared_ptr<ct::core::StateFeedbackController<state_dim, control_dim>> prevController(
            new ct::core::StateFeedbackController<state_dim, control_dim>(newPolicy));
        double time_window = 0.2;
        for (size_t i = 0; i < mpcStateTrajectory.size() - static_cast<size_t>(nloc_settings.computeK(time_window));
             i++)
        {
            double t_forward_start = i * nloc_settings.dt;
            double t_forward_stop = t_forward_start + time_window;

            mpcStateTrajectory.setInterpolationType(ct::core::InterpolationType::LIN);
            ct::core::StateVector<state_dim> start_state = mpcStateTrajectory.eval(t_forward_start);
            ct::core::StateVector<state_dim> ref_end_state = mpcStateTrajectory.eval(t_forward_stop);
            mpcSolver.doForwardIntegration(t_forward_start, t_forward_stop, start_state, prevController);

            // temporarily commented out, as hard to guarantee this condition is met
            // ASSERT_LT((start_state-ref_end_state).array().abs().maxCoeff(), 1e-1);
        }

    } catch (std::exception& e)
    {
        std::cout << "caught exception: " << e.what() << std::endl;
        FAIL();
    }
}


TEST(MPCTestB, NLOC_MPC_DoublePrecision)
{
    typedef tpl::LinearOscillator<double> LinearOscillator;
    typedef tpl::LinearOscillatorLinear<double> LinearOscillatorLinear;

    for (int solverType = 0; solverType <= 0; solverType++)

        try
        {
            Eigen::Vector2d x_final;
            x_final << 20, 0;

            StateVector<state_dim> x0;
            x0.setRandom();

            ct::core::Time timeHorizon = 3.0;

            // set up the Optimal Control Problem
            shared_ptr<ControlledSystem<state_dim, control_dim>> system(new LinearOscillator);
            shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear);
            shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
                tpl::createCostFunctionLinearOscillator<double>(x_final);

            ContinuousOptConProblem<state_dim, control_dim> optConProblem(system, costFunction, analyticLinearSystem);
            optConProblem.setTimeHorizon(timeHorizon);
            optConProblem.setInitialState(x0);


            // FIRST ILQR INSTANCE FOR CALCULATING THE 'PERFECT' INITIAL GUESS

            NLOptConSettings nloc_settings;
            nloc_settings.dt = 0.01;
            nloc_settings.K_sim = 1;
            nloc_settings.K_shot = 1;
            nloc_settings.max_iterations = 10;
            nloc_settings.min_cost_improvement = 1e-10;  // strict bounds to reach a solution very close to optimality
            nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
            nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
            nloc_settings.integrator = ct::core::IntegrationType::EULER;
            nloc_settings.lineSearchSettings.type = LineSearchSettings::TYPE::SIMPLE;
            nloc_settings.nThreads = 1;
            nloc_settings.nThreadsEigen = 1;
            nloc_settings.printSummary = false;
            nloc_settings.debugPrint = false;


            if (solverType == 0)
                nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
            else
                nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;


            int K = nloc_settings.computeK(timeHorizon);  // number of steps


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

            // mpc specific settings
            ct::optcon::mpc_settings settings;
            settings.stateForwardIntegration_ = true;
            settings.stateForwardIntegratorType_ = nloc_settings.integrator;
            settings.stateForwardIntegration_dt_ = nloc_settings.dt;
            settings.postTruncation_ = false;
            settings.measureDelay_ = false;
            settings.fixedDelayUs_ = 100000;  //
            settings.delayMeasurementMultiplier_ = 1.0;
            settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
            settings.coldStart_ = false;
            settings.additionalDelayUs_ = 0;
            settings.useExternalTiming_ = true;


            // Create MPC object
            MPC<NLOptConSolver<state_dim, control_dim>> mpcSolver(optConProblem, nloc_settings_mpc, settings);

            mpcSolver.setInitialGuess(perfectInitController);

            // outputs
            std::vector<ct::core::StateTrajectory<state_dim>>
                stateTrajContainer;  // collection of all state trajectories
            ct::core::StateTrajectory<state_dim> tempStateTraj;
            std::vector<double> timeStamps;                            // collection of all policy-start timestamps
            std::vector<ct::core::StateVector<state_dim>> initStates;  // collection of all initial tests

            int maxNumRuns = 2000;
            int numRuns = 0;


            // timestamp of the new optimal policy
            ct::core::Time ts_newPolicy = 0.0;

            mpcSolver.prepareIteration(0.0);


            for (int i = 0; i < maxNumRuns; i++)
            {
                // we assume to have a perfect initial state (perfect state evolution)
                if (i == 1)
                    x0 = tempStateTraj.eval(1e-6 * settings.fixedDelayUs_);
                else if (i > 0)
                    x0 = tempStateTraj.front();

                // (fake) time which has passed since start of MPC
                double t = i * 1e-6 * settings.fixedDelayUs_;

                // new optimal policy
                ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

                // run one mpc cycle
                bool success = mpcSolver.finishIteration(x0, t, newPolicy, ts_newPolicy);
                mpcSolver.prepareIteration(t);

                tempStateTraj = newPolicy.getReferenceStateTrajectory();

                // save trajectories
                stateTrajContainer.push_back(tempStateTraj);
                timeStamps.push_back(ts_newPolicy);
                initStates.push_back(x0);


                ct::core::Time relTime = ts_newPolicy;
                ct::core::StateVector<state_dim> mpcTrajFirstState = tempStateTraj.front();
                ct::core::StateVector<state_dim> refState = perfectStateTrajectory.eval(relTime);

                // Intuition of this test:
                // The start of every mpc state trajectory has to be close to the initial "perfect" state trajectory.
                // we allow for some tolerance, as the optimal trajectories might slightly change with shrinking time horizon
                ASSERT_LT(std::fabs((refState - mpcTrajFirstState)(0)), 1.0);  // max pos deviation
                ASSERT_LT(std::fabs((refState - mpcTrajFirstState)(1)), 2.0);  // max vel deviation

                if (mpcSolver.timeHorizonReached() || !success)
                    break;

                numRuns++;
            }


            mpcSolver.printMpcSummary();

            ASSERT_GT(numRuns, 10);  // make sure that MPC runs more than 10 times


// The resulting trajectories can be visualized in MATLAB using the script mpc_unittest_plotting.m
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

        } catch (std::exception& e)
        {
            std::cout << "caught exception: " << e.what() << std::endl;
            FAIL();
        }
}


}  // namespace example
}  // namespace optcon
}  // namespace ct
