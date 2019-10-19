/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <chrono>
#include <fenv.h>

#include <gtest/gtest.h>

#include "DiehlSystem.h"
#include "nloc_test_dir.h"


/*!
 * This file implements a NLOC unit tests.
 * For more intuitive examples, visit the tutorial.
 *
 * \example NonlinearSystemTest.h
 *
 *
 */
namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;
using std::shared_ptr;


TEST(NLOCTest, NonlinearSystemAlgorithmComparison)
{
    typedef NLOptConSolver<state_dim, control_dim, 1, 0> NLOptConSolver;

    std::cout << "setting up problem " << std::endl;

    std::string configFile = std::string(NLOC_TEST_DIR) + "/nonlinear/solver.info";
    std::string costFunctionFile = std::string(NLOC_TEST_DIR) + "/nonlinear/cost.info";

    Eigen::Matrix<double, 1, 1> x_0;
    ct::core::loadMatrix(costFunctionFile, "x_0", x_0);

    Eigen::Matrix<double, 1, 1> x_f;
    ct::core::loadMatrix(costFunctionFile, "term1.x_f.weigths.x_des", x_f);

    NLOptConSettings gnms_settings;
    gnms_settings.load(configFile, true, "gnms");

    NLOptConSettings ilqr_settings;
    ilqr_settings.load(configFile, true, "ilqr");

    std::shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new Dynamics);
    std::shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearizedSystem);
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>(costFunctionFile));

    // times
    ct::core::Time tf = 3.0;
    ct::core::loadScalar(configFile, "timeHorizon", tf);

    size_t nSteps = gnms_settings.computeK(tf);

    // provide initial guess
    ControlVector<control_dim> uff_init_guess;
    uff_init_guess << -(x_0(0) + 1) * x_0(0);
    ControlVectorArray<control_dim> u0(nSteps, uff_init_guess);
    StateVectorArray<state_dim> x0(nSteps + 1, x_0);

    FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
    NLOptConSolver::Policy_t initController(x0, u0, u0_fb, gnms_settings.dt);


    // construct single-core single subsystem OptCon Problem
    ContinuousOptConProblem<state_dim, control_dim> optConProblem1(
        tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);
    ContinuousOptConProblem<state_dim, control_dim> optConProblem2(
        tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


    std::cout << "initializing solvers" << std::endl;
    NLOptConSolver gnms(optConProblem1, gnms_settings);
    NLOptConSolver ilqr(optConProblem2, ilqr_settings);


    gnms.configure(gnms_settings);
    gnms.setInitialGuess(initController);

    ilqr.configure(ilqr_settings);
    ilqr.setInitialGuess(initController);


    std::cout << "============ running solvers ==============" << std::endl;

    gnms.solve();

    // print trajectories
    StateTrajectory<state_dim> xRollout_gnms = gnms.getStateTrajectory();
    ControlTrajectory<control_dim> uRollout_gnms = gnms.getControlTrajectory();

    ilqr.solve();

    // print trajectories
    StateTrajectory<state_dim> xRollout_ilqr = ilqr.getStateTrajectory();
    ControlTrajectory<control_dim> uRollout_ilqr = ilqr.getControlTrajectory();

    // Assert that the solutions are equal
    for (size_t i = 0; i < xRollout_gnms.size(); i++)
    {
        ASSERT_NEAR(xRollout_gnms[i](0), xRollout_ilqr[i](0), 1e-4);
    }

    for (size_t i = 0; i < uRollout_ilqr.size(); i++)
    {
        ASSERT_NEAR(uRollout_gnms[i](0), uRollout_ilqr[i](0), 1e-4);
    }
}
}  // namespace example
}  // namespace optcon
}  // namespace ct
