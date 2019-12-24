/*!
 * \example NLOC.cpp
 *
 * This example shows how to use the nonlinear optimal control solvers iLQR, unconstrained Gauss-Newton-Multiple-Shooting (GNMS),
 * as well as the hybrid methods iLQR-GNMS(M), where M denotes the number of multiple-shooting intervals. This example applies
 * them to a simple second-order oscillator.
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


    /* STEP 1-D: initialization with initial state and desired time horizon */

    StateVector<state_dim> x0;
    x0.setRandom();  // in this example, we choose a random initial state x0

    ct::core::Time timeHorizon = 1.0;  // and a final time horizon in [sec]


    // STEP 1-E: create and initialize an "optimal control problem"
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x0, oscillatorDynamics, costFunction, adLinearizer);


    /* STEP 2: set up a nonlinear optimal control solver. */

    /* STEP 2-A: Create the settings.
	 * the type of solver, and most parameters, like number of shooting intervals, etc.,
	 * can be chosen using the following settings struct. Let's use, the iterative
	 * linear quadratic regulator, iLQR, for this example. In the following, we
	 * modify only a few settings, for more detail, check out the NLOptConSettings class. */
    NLOptConSettings nloc_settings;
    nloc_settings.load(ct::optcon::exampleDir + "/nlocSolver.info", true, "ilqr");


    /* STEP 2-B: provide an initial guess */
    // calculate the number of time steps K
    size_t K = nloc_settings.computeK(timeHorizon);

    /* design trivial initial controller for iLQR. Note that in this simple example,
	 * we can simply use zero feedforward with zero feedback gains around the initial position.
	 * In more complex examples, a more elaborate initial guess may be required.*/
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Random());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);


    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, nloc_settings);

    // set the initial guess
    iLQR.setInitialGuess(initController);


    // STEP 3: solve the optimal control problem
    iLQR.solve();


    // STEP 4: retrieve the solution
    ct::core::StateFeedbackController<state_dim, control_dim> solution = iLQR.getSolution();

    // let's plot the output
    plotResultsOscillator<state_dim, control_dim>(solution.x_ref(), solution.uff(), solution.time());
}
