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
    const ct::core::ControlVector<control_dim>& u_lb,
    const ct::core::ControlVector<control_dim>& u_ub)
{
    for (size_t j = 0; j < u.size(); j++)
    {
        for (size_t n = 0; n < control_dim; n++)
        {
            ASSERT_LE(u[j](n), u_ub(n));
            ASSERT_GE(u[j](n), u_lb(n));
        }
    }
}

//! check that state bounds are respected
template <size_t state_dim, size_t control_dim>
void assertStateBounds(const ct::core::StateVectorArray<state_dim>& x,
    const ct::core::StateVector<state_dim>& x_lb,
    const ct::core::StateVector<state_dim>& x_ub)
{
    for (size_t j = 0; j < x.size(); j++)
    {
        for (size_t n = 0; n < state_dim; n++)
        {
            ASSERT_LE(x[j](n), x_ub(n));
            ASSERT_GE(x[j](n), x_lb(n));
        }
    }
}


template <size_t state_dim, size_t control_dim, typename LINEAR_SYSTEM>
void boxConstraintsTest(ct::core::ControlVector<control_dim> u0,
    ct::core::StateVector<state_dim> x0,
    ct::core::StateVector<state_dim> xf,
    ct::core::ControlVector<control_dim> u_lb,
    ct::core::ControlVector<control_dim> u_ub,
    Eigen::Matrix<int, control_dim, 1> u_box_sparsity,
    ct::core::StateVector<state_dim> x_lb,
    ct::core::StateVector<state_dim> x_ub,
    Eigen::Matrix<int, state_dim, 1> x_box_sparsity)
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
        std::cout << " TEST CASE 1: FULL BOX CONSTRAINTS ON CONTROL INPUT " << std::endl;
        std::cout << " ================================================== " << std::endl;
    }

    // initialize the optimal control problems for both solvers
    lqocProblem1->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem2->setFromTimeInvariantLinearQuadraticProblem(x0, u0, discreteExampleSystem, *costFunction, xf, dt);
    lqocProblem1->setControlBoxConstraints(u_lb, u_ub, u_box_sparsity);
    lqocProblem2->setControlBoxConstraints(u_lb, u_ub, u_box_sparsity);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_FALSE(lqocProblem1->isStateBoxConstrained());
    ASSERT_TRUE(lqocProblem1->isControlBoxConstrained());

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
    assertControlBounds<state_dim, control_dim>(uSol_hpipm, u_lb, u_ub);


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
    lqocProblem1->setStateBoxConstraints(x_lb, x_ub, x_box_sparsity);
    lqocProblem2->setStateBoxConstraints(x_lb, x_ub, x_box_sparsity);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isStateBoxConstrained());
    ASSERT_FALSE(lqocProblem1->isControlBoxConstrained());

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
    assertStateBounds<state_dim, control_dim>(xSol_hpipm, x_lb, x_ub);


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

    lqocProblem1->setStateBoxConstraints(x_lb, x_ub, x_box_sparsity);
    lqocProblem2->setStateBoxConstraints(x_lb, x_ub, x_box_sparsity);
    lqocProblem1->setControlBoxConstraints(u_lb, u_ub, u_box_sparsity);
    lqocProblem2->setControlBoxConstraints(u_lb, u_ub, u_box_sparsity);

    // check that constraint configuration is right
    ASSERT_TRUE(lqocProblem1->isConstrained());
    ASSERT_FALSE(lqocProblem1->isGeneralConstrained());
    ASSERT_TRUE(lqocProblem1->isStateBoxConstrained());
    ASSERT_TRUE(lqocProblem1->isControlBoxConstrained());

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
    assertControlBounds<state_dim, control_dim>(uSol_hpipm, u_lb, u_ub);
    assertStateBounds<state_dim, control_dim>(xSol_hpipm, x_lb, x_ub);
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
    ASSERT_FALSE(lqocProblem1->isStateBoxConstrained());
    ASSERT_FALSE(lqocProblem1->isControlBoxConstrained());

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

    // control box constraints
    ct::core::ControlVector<control_dim> u_lb, u_ub;
    u_lb.setConstant(-0.5);
    u_ub.setConstant(0.5);
    Eigen::Matrix<int, control_dim, 1> u_box_sparsity =
        Eigen::Matrix<int, control_dim, 1>::Ones();  // all controls are bounded

    // state box constraints
    ct::core::StateVector<state_dim> x_lb, x_ub;
    x_lb.setConstant(std::numeric_limits<double>::lowest());
    x_ub.setConstant(std::numeric_limits<double>::max());
    Eigen::Matrix<int, state_dim, 1> x_box_sparsity = Eigen::Matrix<int, state_dim, 1>::Zero();
    x_lb(0) = 1.7;
    x_box_sparsity(0) = 1;

    boxConstraintsTest<state_dim, control_dim, example::LinearOscillatorLinear>(
        u0, x0, xf, u_lb, u_ub, u_box_sparsity, x_lb, x_ub, x_box_sparsity);
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

    // control box constraints
    ct::core::ControlVector<control_dim> u_lb, u_ub;
    u_lb.setConstant(-0.5);
    u_ub.setConstant(0.5);
    Eigen::Matrix<int, control_dim, 1> u_box_sparsity;
    u_box_sparsity << 1, 1, 1;

    // state box constraints
    ct::core::StateVector<state_dim> x_lb, x_ub;
    x_lb.setConstant(std::numeric_limits<double>::lowest());
    x_ub.setConstant(std::numeric_limits<double>::max());
    Eigen::Matrix<int, state_dim, 1> x_box_sparsity = Eigen::Matrix<int, state_dim, 1>::Zero();
    x_lb(0) = 1.5;
    x_lb(1) = 1.5;
    x_box_sparsity(0) = 1;
    x_box_sparsity(1) = 1;

    boxConstraintsTest<state_dim, control_dim, LinkedMasses>(
        u0, x0, xf, u_lb, u_ub, u_box_sparsity, x_lb, x_ub, x_box_sparsity);
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
