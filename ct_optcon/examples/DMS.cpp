/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include "exampleDir.h"
#include "plotResultsOscillator.h"

/*!
 * This example shows how to use classical Direct Multiple Shooting with an oscillator system,
 * using IPOPT as NLP solver.
 *
 * \example DMS.cpp
 */
int main(int argc, char** argv)
{
    using namespace ct::optcon;
    using namespace ct::core;

    const size_t state_dim = SecondOrderSystem::STATE_DIM;
    const size_t control_dim = SecondOrderSystem::CONTROL_DIM;


    /**
	 * STEP 1 : set up the optimal control problem
	 * */

    StateVector<state_dim> x_0;
    StateVector<state_dim> x_final;

    x_0 << 0.0, 0.0;
    x_final << 2.0, -1.0;

    double w_n = 0.5;    // oscillator frequency
    double zeta = 0.01;  // oscillator damping

    // create oscillator system
    std::shared_ptr<SecondOrderSystem> oscillator(new SecondOrderSystem(w_n, zeta));


    // load the cost weighting matrices from file and store them in terms. Note that we only use intermediate cost
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    intermediateCost->loadConfigFile(ct::optcon::exampleDir + "/dmsCost.info", "intermediateCost", true);

    // create a cost function and add the terms to it.
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);


    // we include the desired terminal state as a hard constraint
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> finalConstraints(
        new ct::optcon::ConstraintContainerAnalytical<2, 1>());

    std::shared_ptr<TerminalConstraint<state_dim, control_dim>> terminalConstraint(
        new TerminalConstraint<2, 1>(x_final));
    terminalConstraint->setName("TerminalConstraint");
    finalConstraints->addTerminalConstraint(terminalConstraint, true);
    finalConstraints->initialize();


    // define optcon problem and add constraint
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(oscillator, costFunction);
    optConProblem.setInitialState(x_0);
    optConProblem.setGeneralConstraints(finalConstraints);


    /**
	 * STEP 2 : determine solver settings
	 */
    DmsSettings settings;
    settings.N_ = 25;        // number of nodes
    settings.T_ = 5.0;       // final time horizon
    settings.nThreads_ = 4;  // number of threads for multi-threading
    settings.splineType_ = DmsSettings::PIECEWISE_LINEAR;
    settings.costEvaluationType_ = DmsSettings::FULL;  // we evaluate the full cost and use no trapezoidal approximation
    settings.objectiveType_ = DmsSettings::KEEP_TIME_AND_GRID;  // don't optimize the time spacing between the nodes
    settings.h_min_ = 0.1;                         // minimum admissible distance between two nodes in [sec]
    settings.integrationType_ = DmsSettings::RK4;  // type of the shot integrator
    settings.dt_sim_ = 0.01;                       // forward simulation dt
    settings.solverSettings_.solverType_ = NlpSolverType::IPOPT;  // use IPOPT
    settings.absErrTol_ = 1e-8;
    settings.relErrTol_ = 1e-8;

    settings.print();


    /**
	 * STEP 3 : Calculate an appropriate initial guess
	 */
    StateVectorArray<state_dim> x_initguess;
    ControlVectorArray<control_dim> u_initguess;
    DmsPolicy<state_dim, control_dim> initialPolicy;


    x_initguess.resize(settings.N_ + 1, StateVector<state_dim>::Zero());
    u_initguess.resize(settings.N_ + 1, ControlVector<control_dim>::Zero());
    for (size_t i = 0; i < settings.N_ + 1; ++i)
    {
        x_initguess[i] = x_0 + (x_final - x_0) * (i / settings.N_);
    }

    initialPolicy.xSolution_ = x_initguess;
    initialPolicy.uSolution_ = u_initguess;


    /**
	 * STEP 4: solve DMS with IPOPT
	 */

    optConProblem.setTimeHorizon(settings.T_);

    std::shared_ptr<DmsSolver<state_dim, control_dim>> dmsSolver(
        new DmsSolver<state_dim, control_dim>(optConProblem, settings));

    dmsSolver->setInitialGuess(initialPolicy);
    dmsSolver->solve();

    // retrieve the solution
    DmsPolicy<state_dim, control_dim> solution = dmsSolver->getSolution();

    // let's plot the output
    plotResultsOscillator<state_dim, control_dim>(solution.xSolution_, solution.uSolution_, solution.tSolution_);
}
