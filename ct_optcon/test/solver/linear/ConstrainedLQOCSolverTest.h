/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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
    for (size_t j = 0; j < u.size(); j++)
    {
        for (size_t n = 0; n < sparsity.rows(); n++)
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
        for (size_t n = 0; n < sparsity.rows(); n++)
        {
            ASSERT_GE(x[j](sparsity(n) - control_dim), x_lb(n));
            ASSERT_LE(x[j](sparsity(n) - control_dim), x_ub(n));
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
    Eigen::VectorXi x_box_sparsity,
    Eigen::VectorXi x_box_sparsity_terminal)
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
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);

    lqocProblem1->setIntermediateBoxConstraints(nb_u, u_lb, u_ub, u_box_sparsity);
    lqocProblem2->setIntermediateBoxConstraints(nb_u, u_lb, u_ub, u_box_sparsity);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();

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
    hpipmSolver->getFeedback(KSol_hpipm);


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
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);

    lqocProblem1->setIntermediateBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity);
    lqocProblem2->setIntermediateBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity);
    lqocProblem1->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity_terminal);
    lqocProblem2->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity_terminal);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();

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
    hpipmSolver->getFeedback(KSol_hpipm);

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
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);

    // relax box constraints a bit for this test, otherwise there might be no solution
    x_lb.array() -= 1.0;
    x_ub.array() += 1.0;

    // combine the box constraints manually for this test
    int nb_ux = nb_x + nb_u;
    Eigen::VectorXd ux_lb(nb_ux);
    ux_lb << u_lb, x_lb;
    Eigen::VectorXd ux_ub(nb_ux);
    ux_ub << u_ub, x_ub;
    Eigen::VectorXi ux_box_sparsity(nb_ux);
    ux_box_sparsity << u_box_sparsity, x_box_sparsity;

    // set the combined box constraints
    lqocProblem1->setIntermediateBoxConstraints(nb_ux, ux_lb, ux_ub, ux_box_sparsity);
    lqocProblem2->setIntermediateBoxConstraints(nb_ux, ux_lb, ux_ub, ux_box_sparsity);
    lqocProblem1->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity_terminal);
    lqocProblem2->setTerminalBoxConstraints(nb_x, x_lb, x_ub, x_box_sparsity_terminal);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();

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
    hpipmSolver->getFeedback(KSol_hpipm);

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
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem1->setGeneralConstraints(d_lb, d_ub, C, D);
    lqocProblem2->setGeneralConstraints(d_lb, d_ub, C, D);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_TRUE(lqocProblem1->isGeneralConstrained());
    ASSERT_FALSE(lqocProblem1->isBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();

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
    hpipmSolver->getFeedback(KSol_hpipm);


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
    x0 << 2.5, 0.0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(0.0);

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
    x_lb << 1.7;
    x_ub.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXi x_box_sparsity(nb_x);
    x_box_sparsity << 1;
    Eigen::VectorXi x_box_sparsity_terminal(nb_x);
    x_box_sparsity_terminal << 0;

    boxConstraintsTest<state_dim, control_dim, example::LinearOscillatorLinear>(
        u0, x0, xf, nb_u, u_lb, u_ub, u_box_sparsity, nb_x, x_lb, x_ub, x_box_sparsity, x_box_sparsity_terminal);
}

TEST(ConstrainedLQOCSolverTest, BoxConstrTest_medium)
{
    const size_t state_dim = 8;
    const size_t control_dim = 3;

    // nominal control
    ct::core::ControlVector<control_dim> u0;
    u0.setConstant(0.1);

    // initial state
    ct::core::StateVector<state_dim> x0;
    x0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(0.1);

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
    x_lb << 1.5, 1.5;
    x_ub.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXi x_box_sparsity(nb_x);
    x_box_sparsity << 3, 4;
    Eigen::VectorXi x_box_sparsity_terminal(nb_x);
    x_box_sparsity_terminal << 0, 1;

    boxConstraintsTest<state_dim, control_dim, LinkedMasses>(
        u0, x0, xf, nb_u, u_lb, u_ub, u_box_sparsity, nb_x, x_lb, x_ub, x_box_sparsity, x_box_sparsity_terminal);
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
    x0 << 2.5, 0.0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(0.0);

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
    x0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(0.1);

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
    x0 << 2.5, 2.5, 0, 0, 0, 0, 0, 0;

    // desired final state
    ct::core::StateVector<state_dim> xf;
    xf.setConstant(0.1);

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
    x_lb << 0.5, 0.5;
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
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // add and initialize constraint terms
    constraints->addIntermediateConstraint(controlConstraint, verbose);
    constraints->addIntermediateConstraint(stateConstraint, verbose);
    constraints->addTerminalConstraint(stateConstraint, verbose);
    constraints->initialize();

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

    if (verbose)
    {
        std::cout << " ================================================== " << std::endl;
        std::cout << " LQOC PROBLEM USING CONSTRAINT TOOLBOX   " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setZero();
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);

    // evaluate relevant quantities using the constraint toolbox
    int nb_ux_intermediate = constraints->getJacobianStateNonZeroCountIntermediate() +
                             constraints->getJacobianInputNonZeroCountIntermediate();
    ASSERT_EQ(nb_ux_intermediate, 5);
    int nb_x_terminal = constraints->getJacobianStateNonZeroCountTerminal();
    ASSERT_EQ(nb_x_terminal, 2);

    // get bounds
    Eigen::VectorXd ux_lb_intermediate = constraints->getLowerBoundsIntermediate();
    Eigen::VectorXd ux_ub_intermediate = constraints->getUpperBoundsIntermediate();
    Eigen::VectorXd ux_lb_terminal = constraints->getLowerBoundsTerminal();
    Eigen::VectorXd ux_ub_terminal = constraints->getUpperBoundsTerminal();

    // compute sparsity as required by LQOC Solver
    Eigen::VectorXi foo, u_sparsity_intermediate, x_sparsity_intermediate, x_sparsity_terminal;
    constraints->sparsityPatternInputIntermediate(foo, u_sparsity_intermediate);
    constraints->sparsityPatternStateIntermediate(foo, x_sparsity_intermediate);
    Eigen::VectorXi ux_sparsity_intermediate(nb_ux_intermediate);
    x_sparsity_intermediate.array() += control_dim;  // shift indices to match combined decision vector [u, x]
    ux_sparsity_intermediate << u_sparsity_intermediate, x_sparsity_intermediate;

    if (verbose)
        std::cout << "ux_sparsity_intermediate" << ux_sparsity_intermediate.transpose() << std::endl;

    constraints->sparsityPatternStateTerminal(foo, x_sparsity_terminal);
    if (verbose)
        std::cout << "x_sparsity_terminal" << x_sparsity_terminal.transpose() << std::endl;


    // set the combined box constraints to the LQOC problem
    lqocProblem1->setIntermediateBoxConstraints(
        nb_ux_intermediate, ux_lb_intermediate, ux_ub_intermediate, ux_sparsity_intermediate);
    lqocProblem1->setTerminalBoxConstraints(nb_x_terminal, ux_lb_terminal, ux_ub_terminal, x_sparsity_terminal);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isBoxConstrained());

    // set and try to solve the problem for both solvers
    hpipmSolver->configureBoxConstraints(lqocProblem1);
    hpipmSolver->setProblem(lqocProblem1);
    hpipmSolver->initializeAndAllocate();
    hpipmSolver->solve();


    // retrieve solutions from hpipm
    xSol_hpipm = hpipmSolver->getSolutionState();
    uSol_hpipm = hpipmSolver->getSolutionControl();
    hpipmSolver->getFeedback(KSol_hpipm);

    if (verbose)
        printSolution<state_dim, control_dim>(xSol_hpipm, uSol_hpipm, KSol_hpipm);
}
