/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include <chrono>

#include <gtest/gtest.h>

//#define MATLAB
//#define MATLAB_FULL_LOG

//#define DEBUG_PRINT_MP

#include "../testSystems/LinearOscillator.h"

namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;


/*!
 * This test considers a variety of different solver/algorithm options for NLOC,
 * combined with a linear system. We check if the optimization converges within 1 iteration.
 *
 * all with HPIPM or without HPIPM solver
 *
 * todo: all converge in 1st iteration
 *
 */
TEST(LinearSystemsTest, NLOCSolverTest)
{
    typedef NLOptConSolver<state_dim, control_dim, state_dim / 2, state_dim / 2> NLOptConSolver;

    Eigen::Vector2d x_final;
    x_final << 20, 0;


    NLOptConSettings nloc_settings;
    nloc_settings.epsilon = 0.0;
    nloc_settings.max_iterations = 1;
    nloc_settings.recordSmallestEigenvalue = false;
    nloc_settings.min_cost_improvement = 1e-6;
    nloc_settings.fixedHessianCorrection = false;
    nloc_settings.dt = 0.01;
    nloc_settings.K_sim = 2;
    nloc_settings.K_shot = 1;
    nloc_settings.integrator = ct::core::IntegrationType::EULERCT;
    nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
    nloc_settings.printSummary = false;
    nloc_settings.useSensitivityIntegrator = true;

    // loop through all solver classes
    for (int algClass = 0; algClass < NLOptConSettings::NLOCP_ALGORITHM::NUM_TYPES; algClass++)
    {
        nloc_settings.nlocp_algorithm = static_cast<NLOptConSettings::NLOCP_ALGORITHM>(algClass);

        // change between closed-loop and open-loop
        for (int toggleClosedLoop = 0; toggleClosedLoop <= 1; toggleClosedLoop++)
        {
            nloc_settings.closedLoopShooting = (bool)toggleClosedLoop;

            // switch line search on or off
            for (int toggleLS = 0; toggleLS <= 1; toggleLS++)
            {
                nloc_settings.lineSearchSettings.active = (bool)toggleLS;

                // toggle between single and multi-threading
                for (size_t nThreads = 1; nThreads < 5; nThreads = nThreads + 3)
                {
                    // toggle between iLQR/GNMS and hybrid methods with K_shot !=!
                    for (size_t kshot = 1; kshot < 11; kshot = kshot + 9)
                    {
                        nloc_settings.nThreads = nThreads;

                        std::cout << "testing variant with " << nloc_settings.nThreads << " threads, lineSearch "
                                  << toggleLS << " closedLoop " << toggleClosedLoop << ", algClass " << algClass
                                  << std::endl;

                        // start test solver ==============================================================================
                        shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator());
                        shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(
                            new LinearOscillatorLinear());
                        shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
                            tpl::createCostFunctionLinearOscillator<double>(x_final);

                        // times
                        ct::core::Time tf = 1.0;
                        size_t nSteps = std::round(tf / nloc_settings.dt);

                        // provide initial guess
                        StateVector<state_dim> initState;
                        initState.setZero();
                        initState(1) = 1.0;
                        StateVectorArray<state_dim> x0(nSteps + 1, initState);
                        ControlVector<control_dim> uff;
                        uff << kStiffness * initState(0);
                        ControlVectorArray<control_dim> u0(nSteps, uff);


                        FeedbackArray<state_dim, control_dim> u0_fb(
                            nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
                        ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());

                        NLOptConSolver::Policy_t initController(x0, u0, u0_fb, nloc_settings.dt);

                        // construct single-core single subsystem OptCon Problem
                        OptConProblem<state_dim, control_dim> optConProblem(
                            tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


                        std::cout << "initializing gnms solver" << std::endl;
                        NLOptConSolver solver(optConProblem, nloc_settings);

                        solver.configure(nloc_settings);
                        solver.setInitialGuess(initController);
                        solver.runIteration();  // must be converged after 1 iteration
                        solver.runIteration();  // must be converged after 1 iteration

                        const SummaryAllIterations<double>& summary = solver.getBackend()->getSummary();

                        //						ASSERT_TRUE(summary.lx_norms.back() < 1e-11 && summary.lu_norms.back() < 1e-11 && "NLOC should be converged in one iteration");

                        //						ASSERT_TRUE(summary.lx_norms.front() > 1e-11 && summary.lx_norms.front() > 1e-11 && "NLOC should have improved at least once");

                        //						ASSERT_TRUE(summary.defect_l1_norms.back() < 1e-11 && summary.defect_l1_norms.back() < 1e-11 && "NLOC should not have defects in the end");

                        // test trajectories
                        StateTrajectory<state_dim> xRollout = solver.getStateTrajectory();
                        ControlTrajectory<control_dim> uRollout = solver.getControlTrajectory();

                        //					std::cout<<"x final: " << xRollout.back().transpose() << std::endl;
                        //					std::cout<<"u final: " << uRollout.back().transpose() << std::endl;

                        // end test solver ===========================================================================

                    }  // toggle k_shot
                }      // toggle multi-threading / single-threading
            }          // toggle line-search
        }              // toggle closed-loop
    }                  // toggle solver class
}  // end test


}  // namespace example
}  // namespace optcon
}  // namespace ct
