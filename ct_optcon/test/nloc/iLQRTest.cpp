/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <chrono>

// Bring in gtest
#include <gtest/gtest.h>

#include <ct/optcon/optcon.h>

#include "../testSystems/LinearOscillator.h"


/* This test implements a 1-Dimensional horizontally moving point mass with mass 1kg and attached to a spring
 x = [p, pd] // p - position pointing upwards, against gravity, pd - velocity
 dx = f(x,u)
    = [0 1  x  +  [0      +  [0  u
       0 0]        9.81]      1]

 */

namespace ct {
namespace optcon {
namespace example {

using namespace ct::core;
using namespace ct::optcon;

using std::shared_ptr;


TEST(ILQRTest, SystemLinearizationTest)
{
    shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator());
    shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear());
    shared_ptr<LinearSystem<state_dim, control_dim>> numDiffLinearModelGeneral(
        new SystemLinearizer<state_dim, control_dim>(nonlinearSystem, false));
    shared_ptr<LinearSystem<state_dim, control_dim>> numDiffLinearModelSecondOrder(
        new SystemLinearizer<state_dim, control_dim>(nonlinearSystem, true));
    StateVector<state_dim> xRef;
    ControlVector<control_dim> u;
    Time t = 0;

    size_t nTests = 100;
    for (size_t i = 0; i < nTests; i++)
    {
        xRef.setRandom();

        // we have to set u in a way that it makes x and equilibrium
        u(0) = kStiffness * xRef(0);

        StateVector<state_dim> dxNonlinear;
        nonlinearSystem->computeControlledDynamics(xRef, t, u, dxNonlinear);

        StateVector<state_dim> dxLinear;
        analyticLinearSystem->computeControlledDynamics(xRef, t, u, dxLinear);

        StateVector<state_dim> dxLinearNumDiffGeneral;
        numDiffLinearModelGeneral->computeControlledDynamics(xRef, t, u, dxLinearNumDiffGeneral);

        StateVector<state_dim> dxLinearNumDiffSecondOrder;
        numDiffLinearModelGeneral->computeControlledDynamics(xRef, t, u, dxLinearNumDiffSecondOrder);

        ASSERT_LT((dxNonlinear - dxLinear).array().abs().maxCoeff(), 1e-6);

        ASSERT_LT((dxNonlinear - dxLinearNumDiffGeneral).array().abs().maxCoeff(), 1e-6);

        ASSERT_LT((dxNonlinear - dxLinearNumDiffSecondOrder).array().abs().maxCoeff(), 1e-6);
    }
}


TEST(ILQRTestA, InstancesComparison)
{
    try
    {
        typedef NLOptConSolver<state_dim, control_dim> NLOptConSolver;

        std::cout << "setting up problem " << std::endl;

        Eigen::Vector2d x_final;
        x_final << 20, 0;

        NLOptConSettings ilqr_settings;
        ilqr_settings.dt = 0.001;
        ilqr_settings.K_shot = 1;
        ilqr_settings.K_sim = 1;
        ilqr_settings.epsilon = 0.0;
        ilqr_settings.nThreads = 1;
        ilqr_settings.max_iterations = 5;
        ilqr_settings.recordSmallestEigenvalue = false;
        ilqr_settings.fixedHessianCorrection = false;
        ilqr_settings.min_cost_improvement = 1e-12;
        ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
        ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
        ilqr_settings.closedLoopShooting = true;
        ilqr_settings.integrator = ct::core::IntegrationType::EULER;
        ilqr_settings.printSummary = false;
        ilqr_settings.debugPrint = false;
        ilqr_settings.lineSearchSettings.debugPrint = false;


        // copy settings for MP case, but change number of threads
        NLOptConSettings ilqr_settings_mp = ilqr_settings;
        ilqr_settings_mp.nThreads = 4;

        shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator());
        shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear());
        shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
            tpl::createCostFunctionLinearOscillator<double>(x_final);

        // times
        ct::core::Time tf = 3.0;

        // init state
        StateVector<state_dim> x0;
        x0.setRandom();

        // construct single-core single subsystem OptCon Problem
        OptConProblem<state_dim, control_dim> optConProblem(
            tf, x0, nonlinearSystem, costFunction, analyticLinearSystem);

        size_t nSteps = std::round(tf / ilqr_settings.dt);

        std::cout << "initializing ilqr solver" << std::endl;
        NLOptConSolver ilqr(optConProblem, ilqr_settings);
        NLOptConSolver ilqr_mp(optConProblem, ilqr_settings_mp);

        NLOptConSolver ilqr_comp(optConProblem, ilqr_settings);
        NLOptConSolver ilqr_mp_comp(optConProblem, ilqr_settings_mp);

        // provide initial controller
        FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
        ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
        StateVectorArray<state_dim> x_ref(nSteps + 1, StateVector<state_dim>::Zero());
        NLOptConSolver::Policy_t initController(x_ref, u0_ff, u0_fb, ilqr_settings.dt);

        ilqr.configure(ilqr_settings);
        ilqr.setInitialGuess(initController);
        //! check that if retrieving solution now, we exactly get back the init guess.
        NLOptConSolver::Policy_t mirroredInitguess = ilqr.getSolution();
        for (size_t i = 0; i < initController.uff().size(); i++)
        {
            ASSERT_NEAR(mirroredInitguess.uff()[i](0), initController.uff()[i](0), 1e-3);
            ASSERT_NEAR(mirroredInitguess.K()[i](0), initController.K()[i](0), 1e-3);
            ASSERT_NEAR(mirroredInitguess.x_ref()[i](0), initController.x_ref()[i](0), 1e-3);
        }

        ilqr.solve();

        std::cout << "now going into tests" << std::endl;


        size_t nTests = 2;
        for (size_t i = 0; i < nTests; i++)
        {
            bool foundBetter;

            if (i == 0)
            {
                std::cout << "Turning Line-Search off" << std::endl;
                ilqr_settings.lineSearchSettings.active = false;
                ilqr_settings_mp.lineSearchSettings.active = false;
            }
            else
            {
                std::cout << "Turning Line-Search on" << std::endl;
                ilqr_settings.lineSearchSettings.active = true;
                ilqr_settings_mp.lineSearchSettings.active = true;
            }

            ilqr.configure(ilqr_settings);
            ilqr_mp.configure(ilqr_settings_mp);

            ilqr.setInitialGuess(initController);
            ilqr_mp.setInitialGuess(initController);

            ilqr.solve();
            ilqr_mp.solve();

            // make sure this is really the optimal policy and we get a 'false' as return.
            NLOptConSolver::Policy_t optimalPolicy = ilqr.getSolution();
            NLOptConSolver::Policy_t optimalPolicy_mp = ilqr_mp.getSolution();


            // now check against the comparison iLQG instances
            ilqr_comp.configure(ilqr_settings);
            ilqr_mp_comp.configure(ilqr_settings_mp);

            ilqr_comp.setInitialGuess(optimalPolicy);
            ilqr_mp_comp.setInitialGuess(optimalPolicy_mp);

            ilqr_comp.solve();
            ilqr_mp_comp.solve();

            NLOptConSolver::Policy_t optimalPolicy_comp = ilqr_comp.getSolution();
            NLOptConSolver::Policy_t optimalPolicy_mp_comp = ilqr_mp_comp.getSolution();


            // compare controller sizes
            ASSERT_EQ(optimalPolicy_comp.uff().size(), nSteps);

            ASSERT_EQ(optimalPolicy_comp.uff().size(), optimalPolicy.uff().size());

            // compare controller durations
            ASSERT_EQ(optimalPolicy_comp.getFeedforwardTrajectory().duration(),
                optimalPolicy.getFeedforwardTrajectory().duration());


            // compare controllers for single core and mp case
            for (size_t i = 0; i < optimalPolicy_comp.uff().size() - 1; i++)
            {
                ASSERT_NEAR(optimalPolicy_comp.uff()[i](0), optimalPolicy.uff()[i](0), 1e-3);

                ASSERT_NEAR(optimalPolicy_comp.K()[i].array().abs().maxCoeff(),
                    optimalPolicy.K()[i].array().abs().maxCoeff(), 1e-3);

                ASSERT_NEAR(optimalPolicy_mp_comp.uff()[i](0), optimalPolicy_mp.uff()[i](0), 1e-3);

                ASSERT_NEAR(optimalPolicy_mp_comp.K()[i].array().abs().maxCoeff(),
                    optimalPolicy_mp.K()[i].array().abs().maxCoeff(), 1e-3);
            }
        }

    } catch (std::exception& e)
    {
        std::cout << "caught exception: " << e.what() << std::endl;
        FAIL();
    }
}


TEST(ILQRTestB, SingleCoreTest)
{
    try
    {
        typedef NLOptConSolver<state_dim, control_dim> NLOptConSolver;
        typedef StateMatrix<state_dim> state_matrix_t;
        typedef StateControlMatrix<state_dim, control_dim> state_control_matrix_t;

        std::cout << "setting up problem " << std::endl;

        Eigen::Vector2d x_final;
        x_final << 20, 0;

        NLOptConSettings ilqr_settings;
        ilqr_settings.epsilon = 0.0;
        ilqr_settings.nThreads = 1;
        ilqr_settings.max_iterations = 50;
        ilqr_settings.recordSmallestEigenvalue = false;
        ilqr_settings.fixedHessianCorrection = false;
        ilqr_settings.min_cost_improvement = 1e-6;
        ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
        ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
        ilqr_settings.closedLoopShooting = true;
        ilqr_settings.integrator = ct::core::IntegrationType::RK4;
        ilqr_settings.printSummary = false;


        // copy settings for MP case, but change number of threads
        NLOptConSettings ilqr_settings_mp = ilqr_settings;
        ilqr_settings_mp.nThreads = 4;

        shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator());
        shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear());
        shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
            tpl::createCostFunctionLinearOscillator<double>(x_final);

        // times
        ct::core::Time tf = 3.0;

        // init state
        StateVector<state_dim> x0;
        x0.setRandom();

        // construct single-core single subsystem OptCon Problem
        OptConProblem<state_dim, control_dim> optConProblem(
            tf, x0, nonlinearSystem, costFunction, analyticLinearSystem);

        size_t nSteps = std::round(tf / ilqr_settings.dt);

        std::cout << "initializing ilqr solver" << std::endl;
        NLOptConSolver ilqr(optConProblem, ilqr_settings);
        NLOptConSolver ilqr_mp(optConProblem, ilqr_settings_mp);


        // provide initial controller
        FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
        ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
        StateVectorArray<state_dim> x_ref(nSteps + 1, StateVector<state_dim>::Zero());
        NLOptConSolver::Policy_t initController(x_ref, u0_ff, u0_fb, ilqr_settings.dt);

        ilqr.configure(ilqr_settings);
        ilqr.setInitialGuess(initController);

        bool foundBetter = true;

        while (foundBetter)
            foundBetter = ilqr.runIteration();


        size_t nTests = 4;
        for (size_t i = 0; i < nTests; i++)
        {
            if (i == 0)
            {
                ilqr_settings.lineSearchSettings.active = false;
                ilqr_settings_mp.lineSearchSettings.active = false;
            }
            else
            {
                ilqr_settings.lineSearchSettings.active = true;
                ilqr_settings_mp.lineSearchSettings.active = true;
            }

            if (i < 2)
            {
                ilqr_settings.fixedHessianCorrection = false;
                ilqr_settings_mp.fixedHessianCorrection = false;
            }
            else
            {
                ilqr_settings.fixedHessianCorrection = true;
                ilqr_settings_mp.fixedHessianCorrection = true;
            }

            ilqr.configure(ilqr_settings);
            ilqr_mp.configure(ilqr_settings_mp);

            ilqr.setInitialGuess(initController);
            ilqr_mp.setInitialGuess(initController);

            bool foundBetter = true;
            bool foundBetter_mp = true;
            size_t numIterations = 0;

            while (foundBetter)
            {
                // solve

                foundBetter = ilqr.runIteration();
                foundBetter_mp = ilqr_mp.runIteration();

                // test trajectories

                StateTrajectory<state_dim> xRollout = ilqr.getStateTrajectory();
                ControlTrajectory<control_dim> uRollout = ilqr.getControlTrajectory();

                StateTrajectory<state_dim> xRollout_mp = ilqr_mp.getStateTrajectory();
                ControlTrajectory<control_dim> uRollout_mp = ilqr_mp.getControlTrajectory();

                ASSERT_EQ(xRollout.size(), nSteps + 1);
                ASSERT_EQ(uRollout.size(), nSteps);

                ASSERT_EQ(xRollout_mp.size(), nSteps + 1);
                ASSERT_EQ(uRollout_mp.size(), nSteps);


                // test linearization

                core::StateMatrixArray<state_dim> A;
                core::StateControlMatrixArray<state_dim, control_dim> B;
                ilqr.getBackend()->retrieveLastLinearizedModel(A, B);

                core::StateMatrixArray<state_dim> A_mp;
                core::StateControlMatrixArray<state_dim, control_dim> B_mp;
                ilqr_mp.getBackend()->retrieveLastLinearizedModel(A_mp, B_mp);

                ASSERT_EQ(A.size(), nSteps);
                ASSERT_EQ(B.size(), nSteps);

                ASSERT_EQ(A_mp.size(), nSteps);
                ASSERT_EQ(B_mp.size(), nSteps);

                // check integration
                for (size_t j = 0; j < xRollout.size() - 1; j++)
                {
                    ASSERT_LT((xRollout[j] - xRollout_mp[j]).array().abs().maxCoeff(), 1e-10);
                    ASSERT_LT((uRollout[j] - uRollout_mp[j]).array().abs().maxCoeff(), 1e-10);
                }

                // check linearization
                for (size_t j = 0; j < xRollout.size() - 1; j++)
                {
                    state_matrix_t A_analytic;
                    state_control_matrix_t B_analytic;

                    if (ilqr_settings.discretization == NLOptConSettings::APPROXIMATION::FORWARD_EULER)
                    {
                        A_analytic =
                            state_matrix_t::Identity() +
                            ilqr_settings.dt * analyticLinearSystem->getDerivativeState(xRollout[j], uRollout[j], 0);
                        B_analytic =
                            ilqr_settings.dt * analyticLinearSystem->getDerivativeControl(xRollout[j], uRollout[j], 0);
                    }
                    else if (ilqr_settings.discretization == NLOptConSettings::APPROXIMATION::BACKWARD_EULER)
                    {
                        state_matrix_t aNew =
                            ilqr_settings.dt * analyticLinearSystem->getDerivativeState(xRollout[j], uRollout[j], 0);
                        state_matrix_t aNewInv = (state_matrix_t::Identity() - aNew).colPivHouseholderQr().inverse();
                        A_analytic = aNewInv;
                        B_analytic = aNewInv * ilqr_settings.dt *
                                     analyticLinearSystem->getDerivativeControl(xRollout[j], uRollout[j], 0);
                    }
                    else if (ilqr_settings.discretization == NLOptConSettings::APPROXIMATION::TUSTIN)
                    {
                        state_matrix_t aNew = 0.5 * ilqr_settings.dt *
                                              analyticLinearSystem->getDerivativeState(xRollout[j], uRollout[j], 0);
                        state_matrix_t aNewInv = (state_matrix_t::Identity() - aNew).colPivHouseholderQr().inverse();
                        A_analytic = aNewInv * (state_matrix_t::Identity() + aNew);
                        B_analytic = aNewInv * ilqr_settings.dt *
                                     analyticLinearSystem->getDerivativeControl(xRollout[j], uRollout[j], 0);
                    }

                    ASSERT_LT((A[j] - A_analytic).array().abs().maxCoeff(), 1e-6);
                    ASSERT_LT((A_mp[j] - A_analytic).array().abs().maxCoeff(), 1e-6);
                    ASSERT_LT((A_mp[j] - A[j]).array().abs().maxCoeff(), 1e-12);

                    ASSERT_LT((B[j] - B_analytic).array().abs().maxCoeff(), 1e-6);
                    ASSERT_LT((B_mp[j] - B_analytic).array().abs().maxCoeff(), 1e-6);
                    ASSERT_LT((B_mp[j] - B[j]).array().abs().maxCoeff(), 1e-12);
                }

                ASSERT_EQ(foundBetter, foundBetter_mp);

                numIterations++;

                //note: since this is a linear system, it should actually converge in only 1 iteration.

                ASSERT_LT(numIterations, 3);
            }
        }

    } catch (std::exception& e)
    {
        std::cout << "caught exception: " << e.what() << std::endl;
        FAIL();
    }
}


TEST(ILQRTestC, PolicyComparison)
{
    typedef NLOptConSolver<state_dim, control_dim> NLOptConSolver;


    try
    {
        std::cout << "setting up problem " << std::endl;

        Eigen::Vector2d x_final;
        x_final << 20, 0;

        NLOptConSettings ilqr_settings;
        ilqr_settings.epsilon = 0.0;
        ilqr_settings.nThreads = 1;
        ilqr_settings.max_iterations = 50;
        ilqr_settings.recordSmallestEigenvalue = true;
        ilqr_settings.min_cost_improvement = 1e-6;
        ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
        ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
        ilqr_settings.integrator = ct::core::IntegrationType::EULER;
        ilqr_settings.closedLoopShooting = true;
        ilqr_settings.fixedHessianCorrection = false;
        ilqr_settings.printSummary = false;


        NLOptConSettings ilqr_settings_mp = ilqr_settings;
        ilqr_settings_mp.nThreads = 4;


        shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new LinearOscillator());
        shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearOscillatorLinear());
        shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction =
            tpl::createCostFunctionLinearOscillator<double>(x_final);

        // times
        ct::core::Time tf = 3.0;

        // init state
        StateVector<state_dim> x0;
        x0.setRandom();

        // construct single-core single subsystem OptCon Problem
        OptConProblem<state_dim, control_dim> optConProblem(
            tf, x0, nonlinearSystem, costFunction, analyticLinearSystem);

        size_t nSteps = std::round(tf / ilqr_settings.dt);

        std::cout << "initializing ilqr solver" << std::endl;
        NLOptConSolver ilqr(optConProblem, ilqr_settings);
        NLOptConSolver ilqr_mp(optConProblem, ilqr_settings_mp);


        // provide initial controller
        FeedbackArray<state_dim, control_dim> u0_fb(nSteps, FeedbackMatrix<state_dim, control_dim>::Zero());
        ControlVectorArray<control_dim> u0_ff(nSteps, ControlVector<control_dim>::Zero());
        StateVectorArray<state_dim> x_ref(nSteps + 1, StateVector<state_dim>::Zero());
        NLOptConSolver::Policy_t initController(x_ref, u0_ff, u0_fb, ilqr_settings.dt);

        ilqr.configure(ilqr_settings);
        ilqr.setInitialGuess(initController);
        ilqr.solve();

        ilqr_mp.configure(ilqr_settings_mp);
        ilqr_mp.setInitialGuess(initController);
        ilqr_mp.solve();

        size_t nTests = 2;
        for (size_t i = 0; i < nTests; i++)
        {
            if (i == 0)
            {
                ilqr_settings.lineSearchSettings.active = false;
                ilqr_settings_mp.lineSearchSettings.active = false;
            }
            else
            {
                ilqr_settings.lineSearchSettings.active = true;
                ilqr_settings_mp.lineSearchSettings.active = true;
            }

            ilqr.configure(ilqr_settings);
            ilqr_mp.configure(ilqr_settings_mp);

            ilqr.setInitialGuess(initController);
            ilqr_mp.setInitialGuess(initController);

            bool foundBetter = true;
            bool foundBetter_mp = true;
            size_t numIterations = 0;


            while (foundBetter)
            {
                // solve
                foundBetter = ilqr.runIteration();
                foundBetter_mp = ilqr_mp.runIteration();
                return;

                numIterations++;

                // we should converge in way less than 10 iterations
                ASSERT_LT(numIterations, 10);
            }


            // test trajectories
            StateTrajectory<state_dim> xRollout = ilqr.getStateTrajectory();
            StateTrajectory<state_dim> xRollout_mp = ilqr_mp.getStateTrajectory();

            // the optimal controller
            std::shared_ptr<NLOptConSolver::Policy_t> optController(new NLOptConSolver::Policy_t(ilqr.getSolution()));
            std::shared_ptr<NLOptConSolver::Policy_t> optController_mp(
                new NLOptConSolver::Policy_t(ilqr_mp.getSolution()));

            // two test systems
            std::shared_ptr<ControlledSystem<state_dim, control_dim>> testSystem1(new LinearOscillator());
            std::shared_ptr<ControlledSystem<state_dim, control_dim>> testSystem2(new LinearOscillator());

            // set the controller
            testSystem1->setController(optController);
            testSystem2->setController(optController_mp);

            // test integrators, the same as in iLQG
            ct::core::Integrator<state_dim> testIntegrator1(testSystem1, ct::core::IntegrationType::RK4);
            ct::core::Integrator<state_dim> testIntegrator2(testSystem2, ct::core::IntegrationType::RK4);

            // states
            ct::core::StateVector<state_dim> x_test_1 = x0;
            ct::core::StateVector<state_dim> x_test_2 = x0;

            // do forward integration
            double dt_sim = ilqr_settings.getSimulationTimestep();
            testIntegrator1.integrate_n_steps(x_test_1, 0.0, nSteps, dt_sim);
            testIntegrator2.integrate_n_steps(x_test_2, 0.0, nSteps, dt_sim);

            ASSERT_LT((x_test_1 - xRollout.back()).array().abs().maxCoeff(), 0.3);
            ASSERT_LT((x_test_2 - xRollout_mp.back()).array().abs().maxCoeff(), 0.3);
        }

    } catch (std::exception& e)
    {
        std::cout << "caught exception: " << e.what() << std::endl;
        FAIL();
    }
}


}  // namespace example
}  // namespace optcon
}  // namespace ct


/*!
 * This runs the iLQG unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example iLQGCTest.cpp
 */
int main(int argc, char** argv)
{
    using namespace ct::optcon::example;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
