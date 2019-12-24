/*!
 * \example NLOC_boxConstrained.cpp
 *
 * This example shows how to use box constraints alongside NLOC and requires HPIPM to be installed
 * The unconstrained Riccati backward-pass is replaced by a high-performance interior-point
 * constrained linear-quadratic Optimal Control solver.
 *
 */
#include <ct/optcon/optcon.h>
#include "exampleDir.h"
#include "plotResultsOscillator.h"

using namespace ct::core;
using namespace ct::optcon;


int main(int argc, char** argv)
{
    /*get the state and control input dimension of the oscillator. Since we're dealing with a simple oscillator,
	 the state and control dimensions will be state_dim = 2, and control_dim = 1. */
    const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
    const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;


    /* STEP 1: set up the Nonlinear Optimal Control Problem
	 * First of all, we need to create instances of the system dynamics, the linearized system and the cost function. */

    /* STEP 1-A: create a instance of the oscillator dynamics for the optimal control problem.
	 * Please also compare the documentation of SecondOrderSystem.h */
    double w_n = 0.1;
    double zeta = 5.0;
    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
        new ct::core::SecondOrderSystem(w_n, zeta));


    /* STEP 1-B: Although the first order derivatives of the oscillator are easy to derive, let's illustrate the use of the System Linearizer,
	 * which performs numerical differentiation by the finite-difference method. The system linearizer simply takes the
	 * the system dynamics as argument. Alternatively, you could implement your own first-order derivatives by overloading the class LinearSystem.h */
    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(
        new ct::core::SystemLinearizer<state_dim, control_dim>(oscillatorDynamics));


    /* STEP 1-C: create a cost function. We have pre-specified the cost-function weights for this problem in "nlocCost.info".
	 * Here, we show how to create terms for intermediate and final cost and how to automatically load them from the configuration file.
	 * The verbose option allows to print information about the loaded terms on the terminal. */
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    bool verbose = true;
    intermediateCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost.info", "intermediateCost", verbose);
    finalCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost.info", "finalCost", verbose);

    // Since we are using quadratic cost function terms in this example, the first and second order derivatives are immediately known and we
    // define the cost function to be an "Analytical Cost Function". Let's create the corresponding object and add the previously loaded
    // intermediate and final term.
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);
    costFunction->addFinalTerm(finalCost);


    /* STEP 1-D: set up the box constraints for the control input*/
    // input box constraint boundaries with sparsities in constraint toolbox format
    Eigen::VectorXi sp_control(control_dim);
    sp_control << 1;
    Eigen::VectorXd u_lb(control_dim);
    Eigen::VectorXd u_ub(control_dim);
    u_lb.setConstant(-0.5);
    u_ub = -u_lb;

    // constraint terms
    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlInputBound(
        new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub, sp_control));
    controlInputBound->setName("ControlInputBound");

    // input box constraint constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> inputBoxConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // add and initialize constraint terms
    inputBoxConstraints->addIntermediateConstraint(controlInputBound, verbose);
    inputBoxConstraints->initialize();


    /* STEP 1-E: set up the box constraints for the states */
    // state box constraint boundaries with sparsities in constraint toolbox format
    // we put a box constraint on the velocity, hence the overall constraint dimension is 1.
    Eigen::VectorXi sp_state(state_dim);
    sp_state << 0, 1;
    Eigen::VectorXd x_lb(1);
    Eigen::VectorXd x_ub(1);
    x_lb.setConstant(-0.2);
    x_ub = -x_lb;
    // constraint terms
    std::shared_ptr<StateConstraint<state_dim, control_dim>> stateBound(
        new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sp_state));
    stateBound->setName("StateBound");

    // input box constraint constraint container
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> stateBoxConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // add and initialize constraint terms
    stateBoxConstraints->addIntermediateConstraint(stateBound, verbose);
    stateBoxConstraints->initialize();


    /* STEP 1-F: initialization with initial state and desired time horizon */

    StateVector<state_dim> x0;
    x0.setZero();  // in this example, we choose a zero initial state

    ct::core::Time timeHorizon = 3.0;  // and a final time horizon in [sec]


    // STEP 1-G: create and initialize an "optimal control problem"
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x0, oscillatorDynamics, costFunction, adLinearizer);

    // add the box constraints to the optimal control problem
    optConProblem.setInputBoxConstraints(inputBoxConstraints);
    optConProblem.setStateBoxConstraints(stateBoxConstraints);

    /* STEP 2: set up a nonlinear optimal control solver. */

    /* STEP 2-A: Create the settings.
	 * the type of solver, and most parameters, like number of shooting intervals, etc.,
	 * can be chosen using the following settings struct. Let's use, the iterative
	 * linear quadratic regulator, iLQR, for this example. In the following, we
	 * modify only a few settings, for more detail, check out the NLOptConSettings class. */
    NLOptConSettings ilqr_settings;
    ilqr_settings.dt = 0.01;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nThreads = 1;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
    ilqr_settings.lqoc_solver_settings.num_lqoc_iterations = 10;                // number of riccati sub-iterations
    ilqr_settings.printSummary = true;


    /* STEP 2-B: provide an initial guess */
    // calculate the number of time steps K
    size_t K = ilqr_settings.computeK(timeHorizon);

    /* design trivial initial controller for iLQR. Note that in this simple example,
	 * we can simply use zero feedforward with zero feedback gains around the initial position.
	 * In more complex examples, a more elaborate initial guess may be required.*/
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);


    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);

    // set the initial guess
    iLQR.setInitialGuess(initController);


    // STEP 3: solve the optimal control problem
    iLQR.solve();


    // STEP 4: retrieve the solution
    ct::core::StateFeedbackController<state_dim, control_dim> solution = iLQR.getSolution();

    // let's plot the output
    plotResultsOscillator<state_dim, control_dim>(solution.x_ref(), solution.uff(), solution.time());
}
