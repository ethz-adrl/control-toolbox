/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

using namespace ct;
using namespace ct::core;
using namespace ct::optcon;

#include "../../testSystems/LinkedMasses.h"
#include "../../testSystems/LinearOscillator.h"

bool verbose = false;  // optional verbose output

template <size_t state_dim, size_t control_dim>
void printSolution(const ct::core::StateVectorArray<state_dim>& x,
    const ct::core::ControlVectorArray<control_dim>& u,
    const ct::core::FeedbackArray<state_dim, control_dim>& K)
{
    std::cout << "x array:" << std::endl;
    for (size_t j = 0; j < x.size(); j++)
        std::cout << x[j].transpose() << std::endl;

    std::cout << "u array:" << std::endl;
    for (size_t j = 0; j < u.size(); j++)
        std::cout << u[j].transpose() << std::endl;

    std::cout << "K array:" << std::endl;
    for (size_t j = 0; j < K.size(); j++)
        std::cout << K[j] << std::endl << std::endl;
}

//! check that control bounds are respected
template <size_t state_dim, size_t control_dim>
void assertControlBounds(const ct::core::ControlVectorArray<control_dim>& u,
    const Eigen::VectorXi sparsity,
    const Eigen::VectorXd u_lb,
    const Eigen::VectorXd u_ub)
{
    for (int j = 0; j < (static_cast<int>(u.size())); j++)
    {
        for (int n = 0; n < sparsity.rows(); n++)
        {
            ASSERT_GE(u[j](sparsity(n)), u_lb(n));
            ASSERT_LE(u[j](sparsity(n)), u_ub(n));
        }
    }
}

//! check that state bounds are respected
template <size_t state_dim, size_t control_dim>
void assertStateBounds(const ct::core::StateVectorArray<state_dim>& x,
    const Eigen::VectorXi sparsity,
    const Eigen::VectorXd x_lb,
    const Eigen::VectorXd x_ub)
{
    for (size_t j = 0; j < x.size(); j++)
    {
        for (int n = 0; n < sparsity.rows(); n++)
        {
            ASSERT_GE(x[j](sparsity(n)), x_lb(n));
            ASSERT_LE(x[j](sparsity(n)), x_ub(n));
        }
    }
}

template <size_t state_dim, size_t control_dim, typename LINEAR_SYSTEM>
void boxConstraintsTest(ct::core::ControlVector<control_dim> u0,
    ct::core::StateVector<state_dim> x0,
    ct::core::StateVector<state_dim> xf,
    int nb_u,
    Eigen::VectorXd u_lb,
    Eigen::VectorXd u_ub,
    Eigen::VectorXi u_box_sparsity,
    int nb_x,
    Eigen::VectorXd x_lb,
    Eigen::VectorXd x_ub,
    Eigen::VectorXi x_box_sparsity)
{
    const size_t N = 5;
    const double dt = 0.5;

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>);

    NLOptConSettings nloc_settings;
    nloc_settings.lqoc_solver_settings.num_lqoc_iterations = 50;  // allow 50 iterations
    if (verbose)
        nloc_settings.lqoc_solver_settings.lqoc_debug_print = true;
    hpipmSolver->configure(nloc_settings);

    // create linear-quadratic optimal control problem containers
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem2(new LQOCProblem<state_dim, control_dim>(N));


    // create a continuous-time example system and discretize it
    std::shared_ptr<core::LinearSystem<state_dim, control_dim>> exampleSystem(new LINEAR_SYSTEM());
    core::SensitivityApproximation<state_dim, control_dim> discreteExampleSystem(
        dt, exampleSystem, core::SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL);


    // define cost function matrices
    StateMatrix<state_dim> Q;
    Q.setIdentity();
    Q *= 2.0;
    ControlMatrix<control_dim> R;
    R.setIdentity();
    R *= 2 * 2.0;

    // create a cost function
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, xf, u0, xf, Q));

    // solution variables needed later
    ct::core::StateVectorArray<state_dim> x_nom(N, x0);
    ct::core::ControlVectorArray<control_dim> u_nom(N - 1, u0);

    ct::core::StateVectorArray<state_dim> xSol_hpipm;
    ct::core::ControlVectorArray<control_dim> uSol_hpipm;
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm;

    if (verbose)
    {
        std::cout << " ================================================== " << std::endl;
        std::cout << " TEST CASE 1: FULL BOX CONSTRAINTS ON CONTROL INPUT " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);

    lqocProblem1->setInputBoxConstraints(nb_u, u_lb, u_ub, u_box_sparsity, u_nom);
    lqocProblem2->setInputBoxConstraints(nb_u, u_lb, u_ub, u_box_sparsity, u_nom);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isInputBoxConstrained());
    ASSERT_FALSE(lqocProblem1->isStateBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureInputBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();
    hpipmSolver->computeStatesAndControls();
    hpipmSolver->computeFeedbackMatrices();

    try
    {
        hpipmSolver->compute_lv();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "HPIPMSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    try
    {
        gnRiccatiSolver->setProblem(lqocProblem2);
        gnRiccatiSolver->solve();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "GNRiccatiSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    // retrieve solutions from hpipm
    xSol_hpipm = hpipmSolver->getSolutionState();
    uSol_hpipm = hpipmSolver->getSolutionControl();
    KSol_hpipm = hpipmSolver->getSolutionFeedback();


    if (verbose)
        printSolution<state_dim, control_dim>(xSol_hpipm, uSol_hpipm, KSol_hpipm);

    // assert that the imposed boundary constraints are respected by the solution
    assertControlBounds<state_dim, control_dim>(uSol_hpipm, u_box_sparsity, u_lb, u_ub);


    if (verbose)
    {
        std::cout << " ================================================== " << std::endl;
        std::cout << " TEST CASE 2: FULL BOX CONSTRAINTS ON STATE VECTOR  " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setZero();
    lqocProblem2->setZero();
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);

    lqocProblem1->setIntermediateStateBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom);
    lqocProblem2->setIntermediateStateBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom);
    lqocProblem1->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom.back());
    lqocProblem2->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom.back());

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isStateBoxConstrained());
    ASSERT_FALSE(lqocProblem1->isInputBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureStateBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();
    hpipmSolver->computeStatesAndControls();
    hpipmSolver->computeFeedbackMatrices();

    try
    {
        hpipmSolver->compute_lv();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "HPIPMSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    try
    {
        gnRiccatiSolver->setProblem(lqocProblem2);
        gnRiccatiSolver->solve();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "GNRiccatiSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    // retrieve solutions from hpipm
    xSol_hpipm = hpipmSolver->getSolutionState();
    uSol_hpipm = hpipmSolver->getSolutionControl();
    KSol_hpipm = hpipmSolver->getSolutionFeedback();

    if (verbose)
        printSolution<state_dim, control_dim>(xSol_hpipm, uSol_hpipm, KSol_hpipm);

    // assert that the imposed boundary constraints are respected by the solution
    assertStateBounds<state_dim, control_dim>(xSol_hpipm, x_box_sparsity, x_lb, x_ub);


    if (verbose)
    {
        std::cout << " ================================================== " << std::endl;
        std::cout << " TEST CASE 3: BOX CONSTRAINTS ON STATE AND CONTROL  " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setZero();
    lqocProblem2->setZero();
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);

    // relax box constraints a bit for this test, otherwise there might be no solution
    x_lb.array() -= 1.0;
    x_ub.array() += 1.0;


    // set the combined box constraints
    lqocProblem1->setInputBoxConstraints(nb_u, u_lb, u_ub, u_box_sparsity, u_nom);
    lqocProblem2->setInputBoxConstraints(nb_u, u_lb, u_ub, u_box_sparsity, u_nom);
    lqocProblem1->setIntermediateStateBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom);
    lqocProblem2->setIntermediateStateBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom);
    lqocProblem1->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom.back());
    lqocProblem2->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity, x_nom.back());

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isInputBoxConstrained());
    ASSERT_TRUE(lqocProblem1->isStateBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureInputBoxConstraints(lqocProblem1);
    hpipmSolver->configureStateBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();
    hpipmSolver->computeStatesAndControls();
    hpipmSolver->computeFeedbackMatrices();

    try
    {
        hpipmSolver->compute_lv();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "HPIPMSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    try
    {
        gnRiccatiSolver->setProblem(lqocProblem2);
        gnRiccatiSolver->solve();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "GNRiccatiSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    // retrieve solutions from hpipm
    xSol_hpipm = hpipmSolver->getSolutionState();
    uSol_hpipm = hpipmSolver->getSolutionControl();
    KSol_hpipm = hpipmSolver->getSolutionFeedback();

    if (verbose)
        printSolution<state_dim, control_dim>(xSol_hpipm, uSol_hpipm, KSol_hpipm);

    // assert that the imposed boundary constraints are respected by the solution
    assertControlBounds<state_dim, control_dim>(uSol_hpipm, u_box_sparsity, u_lb, u_ub);
    assertStateBounds<state_dim, control_dim>(xSol_hpipm, x_box_sparsity, x_lb, x_ub);
}


template <size_t state_dim, size_t control_dim, typename LINEAR_SYSTEM>
void generalConstraintsTest(ct::core::ControlVector<control_dim> u0,
    ct::core::StateVector<state_dim> x0,
    ct::core::StateVector<state_dim> xf,
    Eigen::VectorXd d_lb,
    Eigen::VectorXd d_ub,
    Eigen::MatrixXd C,
    Eigen::MatrixXd D)
{
    const size_t N = 5;
    const double dt = 0.5;

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>);

    NLOptConSettings nloc_settings;
    nloc_settings.lqoc_solver_settings.num_lqoc_iterations = 50;  // allow 50 iterations
    hpipmSolver->configure(nloc_settings);

    // create linear-quadratic optimal control problem containers
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem2(new LQOCProblem<state_dim, control_dim>(N));

    // create a continuous-time example system and discretize it
    std::shared_ptr<core::LinearSystem<state_dim, control_dim>> exampleSystem(new LINEAR_SYSTEM());
    core::SensitivityApproximation<state_dim, control_dim> discreteExampleSystem(
        dt, exampleSystem, core::SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL);


    // define cost function matrices
    StateMatrix<state_dim> Q;
    Q.setIdentity();
    Q *= 2.0;
    ControlMatrix<control_dim> R;
    R.setIdentity();
    R *= 2 * 2.0;

    // create a cost function
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, xf, u0, xf, Q));

    // solution variables needed later
    ct::core::StateVectorArray<state_dim> xSol_hpipm;
    ct::core::ControlVectorArray<control_dim> uSol_hpipm;
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm;

    if (verbose)
    {
        std::cout << " ================================================== " << std::endl;
        std::cout << " TEST CASE 1: GENERAL INEQUALITY CONSTRAINTS        " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);
    lqocProblem1->setGeneralConstraints(d_lb, d_ub, C, D);
    lqocProblem2->setGeneralConstraints(d_lb, d_ub, C, D);

    // check that constraint configuration is right
    ASSERT_FALSE(lqocProblem1->isInputBoxConstrained());
    ASSERT_FALSE(lqocProblem1->isStateBoxConstrained());
    ASSERT_TRUE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();
    hpipmSolver->computeStatesAndControls();
    hpipmSolver->computeFeedbackMatrices();

    try
    {
        hpipmSolver->compute_lv();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "HPIPMSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    try
    {
        gnRiccatiSolver->setProblem(lqocProblem2);
        gnRiccatiSolver->solve();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "GNRiccatiSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    // retrieve solutions from hpipm
    xSol_hpipm = hpipmSolver->getSolutionState();
    uSol_hpipm = hpipmSolver->getSolutionControl();
    KSol_hpipm = hpipmSolver->getSolutionFeedback();


    if (verbose)
        printSolution<state_dim, control_dim>(xSol_hpipm, uSol_hpipm, KSol_hpipm);
}

TEST(ConstrainedLQOCSolverTest, BoxConstrTest_small)
{
    const size_t state_dim = 2;
    const size_t control_dim = 1;

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0.setConstant(0);

    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 0.0, 0.0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(2.5);

    // dense control box constraints
    int nb_u = control_dim;
    Eigen::VectorXd u_lb(nb_u);
    Eigen::VectorXd u_ub(nb_u);
    u_lb.setConstant(-0.5);
    u_ub.setConstant(0.5);
    Eigen::VectorXi u_box_sparsity(nb_u);
    u_box_sparsity << 0;

    // sparse state box constraints
    int nb_x = 1;
    Eigen::VectorXd x_lb(nb_x);
    Eigen::VectorXd x_ub(nb_x);
    x_lb << -100;
    x_ub.setConstant(1.7);
    Eigen::VectorXi x_box_sparsity(nb_x);
    x_box_sparsity << 0;

    boxConstraintsTest<state_dim, control_dim, example::LinearOscillatorLinear>(
        u0, x0, xf, nb_u, u_lb, u_ub, u_box_sparsity, nb_x, x_lb, x_ub, x_box_sparsity);
}

TEST(ConstrainedLQOCSolverTest, BoxConstrTest_medium)
{
    const size_t state_dim = 8;
    const size_t control_dim = 3;

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0.setConstant(0.0);

    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 0, 0, 0, 0, 0, 0, 0, 0;  // must start at 0

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // dense control box constraints
    int nb_u = control_dim;
    Eigen::VectorXd u_lb(nb_u);
    Eigen::VectorXd u_ub(nb_u);
    u_lb.setConstant(-0.5);
    u_ub.setConstant(0.5);
    Eigen::VectorXi u_box_sparsity(nb_u);
    u_box_sparsity << 0, 1, 2;

    // sparse state box constraints
    int nb_x = 2;
    Eigen::VectorXd x_lb(nb_x);
    Eigen::VectorXd x_ub(nb_x);
    x_lb << -100, -100;
    x_ub.setConstant(1.5);
    Eigen::VectorXi x_box_sparsity(nb_x);
    x_box_sparsity << 0, 1;

    boxConstraintsTest<state_dim, control_dim, LinkedMasses>(
        u0, x0, xf, nb_u, u_lb, u_ub, u_box_sparsity, nb_x, x_lb, x_ub, x_box_sparsity);
}

TEST(ConstrainedLQOCSolverTest, GeneralConstrTest_small)
{
    const size_t state_dim = 2;
    const size_t control_dim = 1;

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0.setConstant(0);

    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 0.0, 0.0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(2.5);

    // general constraints
    // this general constraints is again nothing but an input inequality constraint:
    Eigen::VectorXd d_lb, d_ub;
    d_lb.resize(1);
    d_ub.resize(1);
    d_lb << -0.5;
    d_ub << 0.5;

    Eigen::MatrixXd C, D;
    C.resize(1, 2);
    D.resize(1, 1);
    C.setZero();
    D(0, 0) = 1;

    generalConstraintsTest<state_dim, control_dim, example::LinearOscillatorLinear>(u0, x0, xf, d_lb, d_ub, C, D);
}

TEST(ConstrainedLQOCSolverTest, GeneralConstrTest_medium)
{
    const size_t state_dim = 8;
    const size_t control_dim = 3;

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0.setConstant(0.1);

    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 0.0, 0.0, 0, 0, 0, 0, 0, 0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(1.0);

    // general constraints
    // this general constraints is again nothing but an input inequality constraint:
    Eigen::VectorXd d_lb, d_ub;
    d_lb.resize(control_dim);
    d_ub.resize(control_dim);
    d_lb.setConstant(-0.5);
    d_ub.setConstant(0.5);

    Eigen::MatrixXd C, D;
    C.resize(control_dim, state_dim);
    D.resize(control_dim, control_dim);
    C.setZero();
    D.setIdentity();

    generalConstraintsTest<state_dim, control_dim, LinkedMasses>(u0, x0, xf, d_lb, d_ub, C, D);
}


TEST(ConstrainedLQOCSolverTest, BoxConstraintUsingConstraintToolbox)
{
    const size_t state_dim = 8;
    const size_t control_dim = 3;

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0.setConstant(0.1);

    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 0.0, 0.0, 0, 0, 0, 0, 0, 0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(1.0);

    // input box constraint boundaries with sparsities in constraint toolbox format
    Eigen::VectorXi sp_control(control_dim);
    sp_control << 1, 1, 1;  // note the different way of specifying sparsity
    Eigen::VectorXd u_lb(control_dim);
    Eigen::VectorXd u_ub(control_dim);
    u_lb.setConstant(-0.5);
    u_ub = -u_lb;

    // state box constraint boundaries with sparsities in constraint toolbox format
    Eigen::VectorXi sp_state(state_dim);
    sp_state << 1, 1, 0, 0, 0, 0, 0, 0;  // note the different way of specifying sparsity
    Eigen::VectorXd x_lb(2);
    x_lb << -0.5, -0.5;
    Eigen::VectorXd x_ub(2);
    x_ub.setConstant(std::numeric_limits<double>::max());

    // constrain terms
    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlConstraint(
        new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub, sp_control));
    controlConstraint->setName("ControlInputConstraint");

    std::shared_ptr<StateConstraint<state_dim, control_dim>> stateConstraint(
        new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sp_state));
    stateConstraint->setName("StateConstraint");

    // create constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> input_constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> state_constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // add and initialize constraint terms
    input_constraints->addIntermediateConstraint(controlConstraint, verbose);
    state_constraints->addIntermediateConstraint(stateConstraint, verbose);
    state_constraints->addTerminalConstraint(stateConstraint, verbose);
    input_constraints->initialize();
    state_constraints->initialize();

    //set up lqoc problems and solvers
    const size_t N = 5;
    const double dt = 0.5;

    // create instances of HPIPM and an unconstrained Gauss-Newton Riccati solver
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);

    NLOptConSettings nloc_settings;
    nloc_settings.lqoc_solver_settings.num_lqoc_iterations = 50;  // allow 50 iterations
    if (verbose)
        nloc_settings.lqoc_solver_settings.lqoc_debug_print = true;
    hpipmSolver->configure(nloc_settings);

    // create linear-quadratic optimal control problem containers
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));

    // create a continuous-time example system and discretize it
    std::shared_ptr<core::LinearSystem<state_dim, control_dim>> exampleSystem(new LinkedMasses());
    core::SensitivityApproximation<state_dim, control_dim> discreteExampleSystem(
        dt, exampleSystem, core::SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

    // define cost function matrices
    StateMatrix<state_dim> Q;
    Q.setIdentity();
    Q *= 2.0;
    ControlMatrix<control_dim> R;
    R.setIdentity();
    R *= 2 * 2.0;

    // create a cost function
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, xf, u0, xf, Q));

    // solution variables needed later
    ct::core::StateVectorArray<state_dim> xSol_hpipm;
    ct::core::ControlVectorArray<control_dim> uSol_hpipm;
    ct::core::FeedbackArray<state_dim, control_dim> KSol_hpipm;

    ct::core::StateVectorArray<state_dim> x_nom(N, x0);
    ct::core::ControlVectorArray<control_dim> u_nom(N - 1, u0);

    if (verbose)
    {
        std::cout << " ================================================== " << std::endl;
        std::cout << " LQOC PROBLEM USING CONSTRAINT TOOLBOX   " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setZero();
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, x0, dt);

    // evaluate relevant quantities using the constraint toolbox
    int nb_u_intermediate = input_constraints->getJacobianStateNonZeroCountIntermediate() +
                            input_constraints->getJacobianInputNonZeroCountIntermediate();
    int nb_x_intermediate = state_constraints->getJacobianStateNonZeroCountIntermediate() +
                            state_constraints->getJacobianInputNonZeroCountIntermediate();
    ASSERT_EQ(nb_u_intermediate + nb_x_intermediate, 5);
    int nb_x_terminal = state_constraints->getJacobianStateNonZeroCountTerminal();
    ASSERT_EQ(nb_x_terminal, 2);

    // get bounds
    Eigen::VectorXd u_lb_intermediate = input_constraints->getLowerBoundsIntermediate();
    Eigen::VectorXd u_ub_intermediate = input_constraints->getUpperBoundsIntermediate();
    Eigen::VectorXd x_lb_intermediate = state_constraints->getLowerBoundsIntermediate();
    Eigen::VectorXd x_ub_intermediate = state_constraints->getUpperBoundsIntermediate();
    Eigen::VectorXd x_lb_terminal = state_constraints->getLowerBoundsTerminal();
    Eigen::VectorXd x_ub_terminal = state_constraints->getUpperBoundsTerminal();

    // compute sparsity as required by LQOC Solver
    Eigen::VectorXi foo, u_sparsity_intermediate, x_sparsity_intermediate, x_sparsity_terminal;
    input_constraints->sparsityPatternInputIntermediate(foo, u_sparsity_intermediate);
    state_constraints->sparsityPatternStateIntermediate(foo, x_sparsity_intermediate);
    state_constraints->sparsityPatternStateTerminal(foo, x_sparsity_terminal);
    if (verbose)
    {
        std::cout << "u_sparsity_intermediate" << u_sparsity_intermediate.transpose() << std::endl;
        std::cout << "x_sparsity_intermediate" << x_sparsity_intermediate.transpose() << std::endl;
        std::cout << "x_sparsity_terminal" << x_sparsity_terminal.transpose() << std::endl;
    }


    // set the combined box constraints to the LQOC problem
    lqocProblem1->setInputBoxConstraints(
        nb_u_intermediate, u_lb_intermediate, u_ub_intermediate, u_sparsity_intermediate, u_nom);
    lqocProblem1->setIntermediateStateBoxConstraints(
        nb_x_intermediate, x_lb_intermediate, x_ub_intermediate, x_sparsity_intermediate, x_nom);
    lqocProblem1->setTerminalBoxConstraints(
        nb_x_terminal, x_lb_terminal, x_ub_terminal, x_sparsity_terminal, x_nom.back());

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isInputBoxConstrained());
    ASSERT_TRUE(lqocProblem1->isStateBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureInputBoxConstraints(lqocProblem1);
    hpipmSolver->configureStateBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();
    hpipmSolver->computeStatesAndControls();
    hpipmSolver->computeFeedbackMatrices();
    try
    {
        hpipmSolver->compute_lv();
        ASSERT_TRUE(false);  // should never reach to this point
    } catch (std::exception& e)
    {
        std::cout << "HPIPMSolver failed with exception " << e.what() << std::endl;
        ASSERT_TRUE(true);
    }

    // retrieve solutions from hpipm
    xSol_hpipm = hpipmSolver->getSolutionState();
    uSol_hpipm = hpipmSolver->getSolutionControl();
    KSol_hpipm = hpipmSolver->getSolutionFeedback();

    if (verbose)
        printSolution<state_dim, control_dim>(xSol_hpipm, uSol_hpipm, KSol_hpipm);
}
