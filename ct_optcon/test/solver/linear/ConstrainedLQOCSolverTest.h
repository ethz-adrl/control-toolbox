/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


using namespace ct;
using namespace ct::optcon;

#include "../../testSystems/SpringLoadedMass.h"


/*
 * In this test:
 * todo assure that GNRiccati throws an error if the problem is constrained
 */
//TEST(ConstrainedLQOCSolverTest, BoxConstraintsTest)
void boxConstraintsTest()
{
    const size_t state_dim = 2;
    const size_t control_dim = 1;
    const size_t N = 5;
    const double dt = 0.5;

    bool verbose = true;  // optional verbose output

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>);

    // create linear-quadratic optimal control problem containers
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem2(new LQOCProblem<state_dim, control_dim>(N));


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

    // affine system offset
    ct::core::StateVector<state_dim> b;
    b << 0.1, 0.1;

    // control box constraints
    ct::core::ControlVector<control_dim> u_lb;
    u_lb.setConstant(-0.5);
    ct::core::ControlVector<control_dim> u_ub;
    u_ub.setConstant(0.5);

    // initialize the optimal control problems for both solvers
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, b, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, b, dt);
    lqocProblem1->setControlBoxConstraints(u_lb, u_ub);
    lqocProblem2->setControlBoxConstraints(u_lb, u_ub);

    // check that constraint configuration is right
    //    ASSERT_TRUE(lqocProblem1->isConstrained());
    //    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    //    ASSERT_FALSE(lqocProblem1->isStateBoxConstrained());
    //    ASSERT_TRUE(lqocProblem1->isControlBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->solve();

    try
    {
        gnRiccatiSolver->setProblem(lqocProblem2);
        gnRiccatiSolver->solve();
        //        ASSERT(false); // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "GNRiccatiSolver failed with exception " << e.what() << std::endl;
        //ASSERT(TRUE)
    }


    // retrieve solutions from hpipm
    auto xSol_hpipm = hpipmSolver->getSolutionState();
    auto uSol_hpipm = hpipmSolver->getSolutionControl();
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm;
    hpipmSolver->getFeedback(KSol_hpipm);


    // todo assert that state and control box constraints are met

    std::cout << "x solution from hpipm solver:" << std::endl;
    for (size_t j = 0; j < xSol_hpipm.size(); j++)
    {
        if (verbose)
        {
            std::cout << xSol_hpipm[j].transpose() << std::endl;
        }
    }

    std::cout << "u solution from hpipm solver:" << std::endl;
    for (size_t j = 0; j < uSol_hpipm.size(); j++)
    {
        if (verbose)
        {
            std::cout << uSol_hpipm[j].transpose() << std::endl;
        }
    }

    std::cout << "K solution from hpipm solver:" << std::endl;
    for (size_t j = 0; j < KSol_hpipm.size(); j++)
    {
        if (verbose)
        {
            std::cout << KSol_hpipm[j] << std::endl << std::endl;
        }
    }
}
