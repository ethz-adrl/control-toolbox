/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


using namespace ct;
using namespace ct::optcon;

#include "../../testSystems/SpringLoadedMass.h"


TEST(LQOCSolverTest, compareHPIPMandRiccati)
{
    const size_t state_dim = 2;
    const size_t control_dim = 1;
    const size_t N = 5;
    const double dt = 0.5;

    bool verbose = false;  // optional verbose output

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>);

    // store them, and identifying names, in a vectors
    std::vector<std::shared_ptr<LQOCSolver<state_dim, control_dim>>> lqocSolvers;
    lqocSolvers.push_back(gnRiccatiSolver);
    lqocSolvers.push_back(hpipmSolver);
    std::vector<std::string> solverNames = {"Riccati", "HPIPM"};

    // create linear-quadratic optimal control problem containers
    std::vector<std::shared_ptr<LQOCProblem<state_dim, control_dim>>> problems;
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem2(new LQOCProblem<state_dim, control_dim>(N));

    problems.push_back(lqocProblem1);
    problems.push_back(lqocProblem2);

    // create a continuous-time example system and discretize it
    std::shared_ptr<core::LinearSystem<state_dim, control_dim>> exampleSystem(new example::SpringLoadedMassLinear());
    core::SensitivityApproximation<state_dim, control_dim> discreteExampleSystem(
        dt, exampleSystem, core::SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0 << 0.1;
    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 0.2, 0.1;
    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf << -1, 0;

    // create pointer to a cost function
    auto costFunction = example::createSpringLoadedMassCostFunction(xf);

    ct::core::StateVector<state_dim> b;
    b << 0.1, 0.1;

    // initialize the optimal control problems for both solvers
    problems[0]->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, b, dt);
    problems[1]->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, b, dt);

    // configure box constraints
    try
    {
        // try to give GNRiccati a constraint...
        lqocSolvers[1]->configureBoxConstraints(problems[1]);
        ASSERT_TRUE(false);
    } catch (const std::exception& e)
    {
        // ... should fail
        ASSERT_TRUE(true);
    }
    lqocSolvers[1]->configureBoxConstraints(problems[1]);

    // set the problem pointers
    lqocSolvers[0]->setProblem(problems[0]);
    lqocSolvers[1]->setProblem(problems[1]);

    // solve the problems
    lqocSolvers[0]->solve();
    lqocSolvers[1]->solve();

    // retrieve solutions from both solvers
    auto xSol_riccati = lqocSolvers[0]->getSolutionState();
    auto uSol_riccati = lqocSolvers[0]->getSolutionControl();
    auto xSol_hpipm = lqocSolvers[1]->getSolutionState();
    auto uSol_hpipm = lqocSolvers[1]->getSolutionControl();

    ct::core::FeedbackArray<state_dim, control_dim> KSol_riccati;
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm;
    lqocSolvers[0]->getFeedback(KSol_riccati);
    lqocSolvers[1]->getFeedback(KSol_hpipm);


    for (size_t j = 0; j < xSol_riccati.size(); j++)
    {
        if (verbose)
        {
            std::cout << "x solution from riccati solver:" << std::endl;
            std::cout << xSol_riccati[j].transpose() << std::endl;
            std::cout << "x solution from hpipm solver:" << std::endl;
            std::cout << xSol_hpipm[j].transpose() << std::endl;
        }
        // assert that state trajectories are identical for both solvers
        ASSERT_NEAR((xSol_riccati[j] - xSol_hpipm[j]).array().abs().maxCoeff(), 0.0, 1e-6);
    }

    for (size_t j = 0; j < uSol_riccati.size(); j++)
    {
        if (verbose)
        {
            std::cout << "u solution from riccati solver:" << std::endl;
            std::cout << uSol_riccati[j].transpose() << std::endl;
            std::cout << "u solution from hpipm solver:" << std::endl;
            std::cout << uSol_hpipm[j].transpose() << std::endl;
        }
        // assert that control trajectories are identical for both solvers
        ASSERT_NEAR((uSol_riccati[j] - uSol_hpipm[j]).array().abs().maxCoeff(), 0.0, 1e-6);
    }

    for (size_t j = 0; j < KSol_riccati.size(); j++)
    {
        if (verbose)
        {
            std::cout << "K solution from riccati solver:" << std::endl;
            std::cout << KSol_riccati[j] << std::endl << std::endl;
            std::cout << "K solution from hpipm solver:" << std::endl;
            std::cout << KSol_hpipm[j] << std::endl << std::endl;
        }
        // assert that feedback trajectories are identical for both solvers
        ASSERT_NEAR((KSol_riccati[j] - KSol_hpipm[j]).array().abs().maxCoeff(), 0.0, 1e-6);
    }
}
