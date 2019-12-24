/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
    u0.setZero();  // by definition
    // initial state
    ct::core::StateVector<state_dim> x0;
    x0.setZero();  // by definition
    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf << -1, 0;

    // create pointer to a cost function
    auto costFunction = example::createSpringLoadedMassCostFunction(xf);

    ct::core::StateVector<state_dim> b;
    b << 0.1, 0.1;

    // initialize the optimal control problems for both solvers
    problems[0]->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, b, dt);
    problems[1]->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, b, dt);

    // set the problem pointers
    lqocSolvers[0]->setProblem(problems[0]);
    lqocSolvers[1]->setProblem(problems[1]);

    // allocate memory (if required)
    lqocSolvers[0]->initializeAndAllocate();
    lqocSolvers[1]->initializeAndAllocate();

    // solve the problems...
    lqocSolvers[0]->solve();
    lqocSolvers[1]->solve();

    // postprocess data
    lqocSolvers[0]->computeStatesAndControls();
    lqocSolvers[0]->computeFeedbackMatrices();
    lqocSolvers[0]->compute_lv();
    lqocSolvers[1]->computeStatesAndControls();
    lqocSolvers[1]->computeFeedbackMatrices();
    lqocSolvers[1]->compute_lv();

    // retrieve solutions from both solvers
    auto xSol_riccati = lqocSolvers[0]->getSolutionState();
    auto uSol_riccati = lqocSolvers[0]->getSolutionControl();
    ct::core::FeedbackArray<state_dim, control_dim> KSol_riccati = lqocSolvers[0]->getSolutionFeedback();
    ct::core::ControlVectorArray<control_dim> lv_sol_riccati = lqocSolvers[0]->get_lv();
    auto xSol_hpipm = lqocSolvers[1]->getSolutionState();
    auto uSol_hpipm = lqocSolvers[1]->getSolutionControl();
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm = lqocSolvers[1]->getSolutionFeedback();
    ct::core::ControlVectorArray<control_dim> lv_sol_hpipm = lqocSolvers[1]->get_lv();


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

    for (size_t j = 0; j < lv_sol_riccati.size(); j++)
    {
        if (verbose)
        {
            std::cout << "lv solution from riccati solver:" << std::endl;
            std::cout << lv_sol_riccati[j] << std::endl << std::endl;
            std::cout << "lv solution from hpipm solver:" << std::endl;
            std::cout << lv_sol_hpipm[j] << std::endl << std::endl;
        }
        // assert that feedforward increments are identical for both solvers
        ASSERT_NEAR((lv_sol_riccati[j] - lv_sol_hpipm[j]).array().abs().maxCoeff(), 0.0, 1e-6);
    }
}
