/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
    x0.setZero();  // by definition

    // define a desired terminal state
    StateVector<state_dim> stateOffset;
    stateOffset.setConstant(0.1);

    // define a nominal control
    ControlVector<control_dim> u0;
    u0.setZero();  // by definition

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
    lqocProblem_hpipm->setFromTimeInvariantLinearQuadraticProblem(discretizedSystem, costFunction, stateOffset, dt);
    lqocProblem_gnriccati->setFromTimeInvariantLinearQuadraticProblem(discretizedSystem, costFunction, stateOffset, dt);


    // create hpipm solver instance, set and solve problem
    ct::optcon::HPIPMInterface<state_dim, control_dim> hpipm;
    hpipm.setProblem(lqocProblem_hpipm);
    hpipm.solve();
    hpipm.computeStatesAndControls();
    hpipm.computeFeedbackMatrices();
    hpipm.compute_lv();

    // create GNRiccati solver instance, set and solve problem
    ct::optcon::GNRiccatiSolver<state_dim, control_dim> gnriccati;
    gnriccati.setProblem(lqocProblem_gnriccati);
    gnriccati.solve();
    gnriccati.computeStatesAndControls();
    gnriccati.computeFeedbackMatrices();
    gnriccati.compute_lv();

    // compute and retrieve solutions
    ct::core::StateVectorArray<state_dim> x_sol_hpipm = hpipm.getSolutionState();
    ct::core::StateVectorArray<state_dim> x_sol_gnrccati = gnriccati.getSolutionState();
    ct::core::ControlVectorArray<control_dim> u_sol_hpipm = hpipm.getSolutionControl();
    ct::core::ControlVectorArray<control_dim> u_sol_gnrccati = gnriccati.getSolutionControl();
    ct::core::FeedbackArray<state_dim, control_dim> K_sol_hpipm = hpipm.getSolutionFeedback();
    ct::core::FeedbackArray<state_dim, control_dim> K_sol_gnriccati = gnriccati.getSolutionFeedback();
    ct::core::ControlVectorArray<control_dim> lv_sol_hpipm = hpipm.get_lv();
    ct::core::ControlVectorArray<control_dim> lv_sol_gnriccati = gnriccati.get_lv();

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

    // assert that feedback matrices are the same
    for (size_t i = 0; i < K_sol_hpipm.size(); i++)
    {
        ASSERT_LT((K_sol_hpipm[i] - K_sol_gnriccati[i]).array().abs().maxCoeff(), 1e-6);
    }

    // assert that feedforward-increment is the same
    for (size_t i = 0; i < lv_sol_hpipm.size(); i++)
    {
        ASSERT_LT((lv_sol_hpipm[i] - lv_sol_gnriccati[i]).array().abs().maxCoeff(), 1e-6);
    }
}
