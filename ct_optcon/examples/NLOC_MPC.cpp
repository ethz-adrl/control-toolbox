
#include <ct/optcon/optcon.h>
#include "exampleDir.h"

using namespace ct::core;
using namespace ct::optcon;

/*!
 * This tutorial example shows how to use the MPC class. In the CT, every optimal control solver can be wrapped into the MPC-class,
 * allowing for very rapid prototyping of different MPC applications.
 * In this example, we apply iLQR-MPC to a simple second order system, a damped oscillator.
 * This tutorial builds up on the example NLOC.cpp, please consider this one as well.
 *
 * \example NLOC_MPC.cpp
 */
int main(int argc, char** argv)
{
    /* PRELIMINIARIES, see example NLOC.cpp */

    const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
    const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;

    double w_n = 0.1;
    double zeta = 5.0;
    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
        new ct::core::SecondOrderSystem(w_n, zeta));

    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(
        new ct::core::SystemLinearizer<state_dim, control_dim>(oscillatorDynamics));

    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    bool verbose = true;
    intermediateCost->loadConfigFile(ct::optcon::exampleDir + "/mpcCost.info", "intermediateCost", verbose);
    finalCost->loadConfigFile(ct::optcon::exampleDir + "/mpcCost.info", "finalCost", verbose);

    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);
    costFunction->addFinalTerm(finalCost);

    StateVector<state_dim> x0;
    x0.setRandom();

    ct::core::Time timeHorizon = 3.0;

    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x0, oscillatorDynamics, costFunction, adLinearizer);


    NLOptConSettings ilqr_settings;
    ilqr_settings.dt = 0.01;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::
        GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = true;

    size_t K = ilqr_settings.computeK(timeHorizon);

    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);


    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);

    // set the initial guess
    iLQR.setInitialGuess(initController);


    // we solve the optimal control problem and retrieve the solution
    iLQR.solve();
    ct::core::StateFeedbackController<state_dim, control_dim> initialSolution = iLQR.getSolution();


    /*  MPC-EXAMPLE
	 * we store the initial solution obtained from solving the initial optimal control problem,
	 * and re-use it to initialize the MPC solver in the following. */

    /* STEP 1: first, we set up an MPC instance for the iLQR solver and configure it. Since the MPC
	 * class is wrapped around normal Optimal Control Solvers, we need to different kind of settings,
	 * those for the optimal control solver, and those specific to MPC: */

    // 1) settings for the iLQR instance used in MPC. Of course, we use the same settings
    // as for solving the initial problem ...
    NLOptConSettings ilqr_settings_mpc = ilqr_settings;
    // ... however, in MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
    ilqr_settings_mpc.max_iterations = 1;
    // and we limited the printouts, too.
    ilqr_settings_mpc.printSummary = false;


    // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
    ct::optcon::mpc_settings mpc_settings;
    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
    mpc_settings.coldStart_ = false;


    // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
    MPC<NLOptConSolver<state_dim, control_dim>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);

    // initialize it using the previously computed initial controller
    ilqr_mpc.setInitialGuess(initialSolution);


    /* STEP 3: running MPC
	 * Here, we run the MPC loop. Note that the general underlying idea is that you receive a state-estimate
	 * together with a time-stamp from your robot or system. MPC needs to receive both that time information and
	 * the state from your control system. Here, "simulate" the time measurement using std::chrono and wrap
	 * everything into a for-loop.
	 * The basic idea of operation is that after receiving time and state information, one executes the finishIteration() method of MPC.
	 */
    auto start_time = std::chrono::high_resolution_clock::now();


    // limit the maximum number of runs in this example
    size_t maxNumRuns = 100;

    std::cout << "Starting to run MPC" << std::endl;

    for (size_t i = 0; i < maxNumRuns; i++)
    {
        // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
        if (i > 0)
            x0 = 0.1 * StateVector<state_dim>::Random();

        // time which has passed since start of MPC
        auto current_time = std::chrono::high_resolution_clock::now();
        ct::core::Time t =
            1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

        // prepare mpc iteration
        ilqr_mpc.prepareIteration(t);

        // new optimal policy
        ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

        // timestamp of the new optimal policy
        ct::core::Time ts_newPolicy;

        current_time = std::chrono::high_resolution_clock::now();
        t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
        bool success = ilqr_mpc.finishIteration(x0, t, newPolicy, ts_newPolicy);

        // we break the loop in case the time horizon is reached or solve() failed
        if (ilqr_mpc.timeHorizonReached() | !success)
            break;
    }


    // the summary contains some statistical data about time delays, etc.
    ilqr_mpc.printMpcSummary();
}
