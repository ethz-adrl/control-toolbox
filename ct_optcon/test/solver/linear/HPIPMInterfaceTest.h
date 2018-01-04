/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This unit test compares HPIPM and the custom GNRiccati solver using custom defined system
 * with state dimension 8 and control dimension 3.
 *
 * \warning This example is not intuitive. For a better introduction into the solver-framework,
 * visit the tutorial.
 */

#include "../../testSystems/LinkedMasses.h"

TEST(HPIPMInterfaceTest, compareSolvers)
{
    const size_t state_dim = 8;
    const size_t control_dim = 3;

    int N = 5;
    double dt = 0.5;

    typedef ct::optcon::LQOCProblem<state_dim, control_dim> LQOCProblem_t;
    std::shared_ptr<LQOCProblem_t> lqocProblem_hpipm(new LQOCProblem_t(N));
    std::shared_ptr<LQOCProblem_t> lqocProblem_gnriccati(new LQOCProblem_t(N));

    // define an initial state
    StateVector<state_dim> x0;
    x0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // define a desired terminal state
    StateVector<state_dim> stateOffset;
    stateOffset.setConstant(0.1);

    // define a nominal control
    ControlVector<control_dim> u0;
    u0.setConstant(-0.1);

    // define cost function matrices
    StateMatrix<state_dim> Q;
    Q.setIdentity();
    Q *= 2.0;
    ControlMatrix<control_dim> R;
    R.setIdentity();
    R *= 2 * 2.0;

    // create a cost function
    ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim> costFunction(
        Q, R, -stateOffset, u0, -stateOffset, Q);

    // create a continuous-time example system and discretize it
    std::shared_ptr<ct::core::LinearSystem<state_dim, control_dim>> system(new LinkedMasses());
    ct::core::SensitivityApproximation<state_dim, control_dim> discretizedSystem(
        dt, system, ct::optcon::NLOptConSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

    // initialize the linear quadratic optimal control problems
    lqocProblem_hpipm->setFromTimeInvariantLinearQuadraticProblem(
        x0, u0, discretizedSystem, costFunction, stateOffset, dt);
    lqocProblem_gnriccati->setFromTimeInvariantLinearQuadraticProblem(
        x0, u0, discretizedSystem, costFunction, stateOffset, dt);


    // create hpipm solver instance, set and solve problem
    ct::optcon::HPIPMInterface<state_dim, control_dim> hpipm;
    hpipm.setProblem(lqocProblem_hpipm);
    hpipm.solve();

    // create GNRiccati solver instance, set and solve problem
    ct::optcon::GNRiccatiSolver<state_dim, control_dim> gnriccati;
    gnriccati.setProblem(lqocProblem_gnriccati);
    gnriccati.solve();

    // retrieve solutions
    ct::core::StateVectorArray<state_dim> x_sol_hpipm = hpipm.getSolutionState();
    ct::core::StateVectorArray<state_dim> x_sol_gnrccati = gnriccati.getSolutionState();
    ct::core::ControlVectorArray<control_dim> u_sol_hpipm = hpipm.getSolutionControl();
    ct::core::ControlVectorArray<control_dim> u_sol_gnrccati = gnriccati.getSolutionControl();

    // asser that the solution sizes the same
    ASSERT_EQ(x_sol_hpipm.size(), x_sol_gnrccati.size());
    ASSERT_EQ(u_sol_hpipm.size(), u_sol_gnrccati.size());

    // assert that states are the same
    for (size_t i = 0; i < x_sol_hpipm.size(); i++)
    {
        ASSERT_LT((x_sol_hpipm[i] - x_sol_gnrccati[i]).array().abs().maxCoeff(), 1e-6);
    }

    // assert that controls are the same
    for (size_t i = 0; i < u_sol_hpipm.size(); i++)
    {
        ASSERT_LT((u_sol_hpipm[i] - u_sol_gnrccati[i]).array().abs().maxCoeff(), 1e-6);
    }
}
