/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <chrono>
#include <fenv.h>

#include <gtest/gtest.h>

//#define MATLAB
//#define MATLAB_FULL_LOG

#include "nloc_test_dir.h"


/*!
 * This file implements a NLOC unit tests.
 * For more intuitive examples, visit the tutorial.
 *
 * \example NonlinearSystemTest.h
 *
 *
 */

/*
 * This test implements a 1-Dimensional horizontally moving point mass with mass 1kg and attached to a spring
 * x = [p, pd] // p - position pointing upwards, against gravity, pd - velocity
 * dx = f(x,u)
 *   = [0 1  x  +  [0      +  [0  u
 *      0 0]        9.81]      1]
 */
namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;

const size_t state_dim = 1;    // position, velocity
const size_t control_dim = 1;  // force

//! Dynamics class for the GNMS unit test
class Dynamics : public ControlledSystem<state_dim, control_dim>
{
public:
    Dynamics() : ControlledSystem<state_dim, control_dim>(SYSTEM_TYPE::SECOND_ORDER) {}
    void computeControlledDynamics(const StateVector<state_dim>& state,
        const Time& t,
        const ControlVector<control_dim>& control,
        StateVector<state_dim>& derivative) override
    {
        derivative(0) = (1.0 + state(0)) * state(0) + control(0);
    }

    Dynamics* clone() const override { return new Dynamics(); };
};

//! Linear system class for the GNMS unit test
class LinearizedSystem : public LinearSystem<state_dim, control_dim>
{
public:
    state_matrix_t A_;
    state_control_matrix_t B_;


    const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        A_ << 1 + 2 * x(0);
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x,
        const ControlVector<control_dim>& u,
        const double t = 0.0) override
    {
        B_ << 1;
        return B_;
    }

    LinearizedSystem* clone() const override { return new LinearizedSystem(); }
};


TEST(NLOCTest, NonlinearSystemTest)
{
    typedef NLOptConSolver<state_dim, control_dim, 1, 0> NLOptConSolver;

    std::cout << "setting up problem " << std::endl;

    std::string configFile = std::string(NLOC_TEST_DIR) + "/config/solver.info";
    std::string costFunctionFile = std::string(NLOC_TEST_DIR) + "/config/cost.info";

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

    int initType = 0;
    ct::core::loadScalar(configFile, "initType", initType);

    switch (initType)
    {
        case 0:  // zero
            break;

        case 1:  // linear
        {
            for (size_t i = 0; i < nSteps + 1; i++)
            {
                x0[i] = x_0 + (x_f - x_0) * double(i) / double(nSteps);
            }
            break;
        }
        case 2:  // integration
        {
            shared_ptr<ControlledSystem<state_dim, control_dim>> systemForInit(new Dynamics);
            ct::core::Integrator<state_dim> integratorForInit(systemForInit, ilqr_settings.integrator);
            x0[0] = x_0;
            for (size_t i = 1; i < nSteps + 1; i++)
            {
                x0[i] = x0[i - 1];
                double dt_sim = gnms_settings.getSimulationTimestep();
                integratorForInit.integrate_n_steps(x0[i], 0, 1, dt_sim);
            }
            break;
        }
        case 3:  // random
        {
            for (size_t i = 1; i < nSteps + 1; i++)
            {
                x0[i].setRandom();
            }
            break;
        }
        case 4:  // zero
        {
            for (size_t i = 1; i < nSteps + 1; i++)
            {
                x0[i].setZero();
            }
            break;
        }
        default:
        {
            throw std::runtime_error("illegal init type");
            break;
        }
    }


    FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
    NLOptConSolver::Policy_t initController(x0, u0, u0_fb, gnms_settings.dt);

    // construct single-core single subsystem OptCon Problem
    OptConProblem<state_dim, control_dim> optConProblem1(
        tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);
    OptConProblem<state_dim, control_dim> optConProblem2(
        tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


    std::cout << "initializing solvers" << std::endl;
    NLOptConSolver gnms(optConProblem1, gnms_settings);
    NLOptConSolver ilqr(optConProblem2, ilqr_settings);


    gnms.configure(gnms_settings);
    gnms.setInitialGuess(initController);

    ilqr.configure(ilqr_settings);
    ilqr.setInitialGuess(initController);


    std::cout << "============ running solver 1 ==============" << std::endl;

    int numIterations = 0;

    while (numIterations < gnms_settings.max_iterations)
    {
        gnms.runIteration();

        // test trajectories
        StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
        ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

        numIterations++;

        std::cout << "x final GNMS: " << xRollout.back().transpose() << std::endl;
        std::cout << "u final GNMS: " << uRollout.back().transpose() << std::endl;
    }

    gnms.logSummaryToMatlab("gnmsSummary");

    std::cout << "============ running solver 2 ==============" << std::endl;

    numIterations = 0;
    while (numIterations < ilqr_settings.max_iterations)
    {
        ilqr.runIteration();

        // test trajectories
        StateTrajectory<state_dim> xRollout = ilqr.getStateTrajectory();
        ControlTrajectory<control_dim> uRollout = ilqr.getControlTrajectory();

        numIterations++;

        std::cout << "x final iLQG: " << xRollout.back().transpose() << std::endl;
        std::cout << "u final iLQG: " << uRollout.back().transpose() << std::endl;
    }

    ilqr.logSummaryToMatlab("ilqrSummary");
}
}
}
}
