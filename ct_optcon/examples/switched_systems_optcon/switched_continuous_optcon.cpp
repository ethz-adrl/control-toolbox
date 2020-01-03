/*!
 * \example switched_continuous_optcon.cpp
 *
 * This example shows how to use switched systems and constraints together with ct_optcon
 * The problem is derived from Example 3 in http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1259455
 *
 */

#include <ct/optcon/optcon.h>
#include "TestLinearSystem.h"
#include "StateSumConstraint.h"
#include "plotResultsSwitched.h"

using namespace ct;
using namespace ct::core;
using namespace ct::optcon;
using std::shared_ptr;

int main(int argc, char** argv)
{
    /* STEP 1: set up the Nonlinear Optimal Control Problem
    * First of all, we need to create instances of the system dynamics, the linearized system and the cost function. */

    /* STEP 1-A: Aliases
    * We create aliases for the system types to be used in this problem */
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;
    using System = TestLinearSystem;
    using SwitchedSystem = SwitchedControlledSystem<STATE_DIM, CONTROL_DIM>;
    using SystemPtr = SwitchedSystem::SystemPtr;
    using SwitchedSystems = SwitchedSystem::SwitchedSystems;
    using ConstantController = ConstantController<STATE_DIM, CONTROL_DIM>;
    using Controller = std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM>>;

    using SwitchedLinearSystem = SwitchedLinearSystem<STATE_DIM, CONTROL_DIM>;
    using SystemLinearizer = SystemLinearizer<STATE_DIM, CONTROL_DIM>;
    using LinearizerSystemPtr = SwitchedLinearSystem::LinearSystemPtr;
    using SwitchedLinearSystems = SwitchedLinearSystem::SwitchedLinearSystems;

    /* STEP 1-B: Create a mode sequence
    * Then we set the switching time, time horizon, and schedule of the systems in the mode sequence.
    * The switching time is not optimized and therefore set to the optimal switching time according to the paper */
    double switchTime = 1.1624;
    double timeHorizon = 2.0;
    ContinuousModeSequence modeSequence;
    modeSequence.addPhase(0, switchTime);                // phase 0, t in [0, t)
    modeSequence.addPhase(1, timeHorizon - switchTime);  // phase 1, t in [t, T)

    /* STEP 1-C: create a cost function.
    * We specify a quadratic penalty on a desired final state and quadratic costs on the inputs */
    Eigen::Matrix<double, STATE_DIM, 1> x_nominal, x_final;
    Eigen::Matrix<double, CONTROL_DIM, 1> u_nominal;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q, Q_final;
    Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R;
    x_nominal.setZero();
    x_final << 10.0, 6.0;
    u_nominal.setZero();
    Q.setZero();
    Q_final.setIdentity();
    R.setIdentity();

    std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> quadraticCostFunction(
        new CostFunctionQuadraticSimple<STATE_DIM, CONTROL_DIM>(Q, R, x_nominal, u_nominal, x_final, Q_final));

    /* STEP 1-D: Create system dynamics
    * Two linear systems are created, linearized and combined into switched systems */
    System::state_matrix_t A1_continuous, A2_continuous;
    A1_continuous << 1.5, 0.0, 0.0, 1.0;
    A2_continuous << 0.5, 0.866, 0.866, -0.5;
    System::state_control_matrix_t B1_continuous, B2_continuous;
    B1_continuous << 1.0, 1.0;
    B2_continuous << 1.0, 1.0;
    SystemPtr sysPtr1(new System(A1_continuous, B1_continuous));
    SystemPtr sysPtr2(new System(A2_continuous, B2_continuous));
    SwitchedSystems switchedSystems;
    switchedSystems.push_back(sysPtr1);
    switchedSystems.push_back(sysPtr2);

    // Setup Constant Controller
    System::control_vector_t u0;
    u0.setZero();
    Controller controller(new ConstantController(u0));

    // Linearization
    sysPtr1->setController(controller);
    sysPtr2->setController(controller);
    LinearizerSystemPtr linSys1(new SystemLinearizer(sysPtr1));
    LinearizerSystemPtr linSys2(new SystemLinearizer(sysPtr2));
    SwitchedLinearSystems switchedLinearSystems;
    switchedLinearSystems.push_back(linSys1);
    switchedLinearSystems.push_back(linSys2);

    // Construct Switched Continuous System and its linearizations
    std::shared_ptr<SwitchedSystem> switchedSystem(new SwitchedSystem(switchedSystems, modeSequence, controller));
    std::shared_ptr<SwitchedLinearSystem> switchedLinearSystem(
        new SwitchedLinearSystem(switchedLinearSystems, modeSequence));

    // Set initial conditions
    System::state_vector_t x0;
    x0 << 1.0, 1.0;

    /* STEP 1-E: Create constraints
    * Two constraints are created, linearized, and combined into a switched constraint
    * Both are a sum of state constraint, but the bounds for each phase are different */
    std::shared_ptr<StateSumConstraint> phase1Constraint(new StateSumConstraint(-1e20, 7.0));
    std::shared_ptr<StateSumConstraint> phase2Constraint(new StateSumConstraint(7.0, 1e20));

    // Linearized constraints
    bool verbose = false;
    std::shared_ptr<ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>> generalConstraints_1(
        new ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>());
    generalConstraints_1->addIntermediateConstraint(phase1Constraint, verbose);

    std::shared_ptr<ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>> generalConstraints_2(
        new ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>());
    generalConstraints_2->addIntermediateConstraint(phase2Constraint, verbose);

    // Switched constraints
    SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM>::SwitchedLinearConstraintContainers
        switchedConstraintContainers;
    switchedConstraintContainers.push_back(generalConstraints_1);
    switchedConstraintContainers.push_back(generalConstraints_2);

    std::shared_ptr<SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM>> switchedConstraints(
        new SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM>(switchedConstraintContainers, modeSequence));

    switchedConstraints->initialize();

    /* STEP 2: set up a nonlinear optimal control solver. */

    /* STEP 2-A: Create the settings.
    * the type of solver, and most parameters, like number of shooting intervals, etc.,
    * can be chosen using the following settings struct. For more detail, check out the NLOptConSettings class. */
    NLOptConSettings ilqr_settings;
    ilqr_settings.dt = 0.001;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 100;
    ilqr_settings.min_cost_improvement = 1e-6;
    ilqr_settings.meritFunctionRhoConstraints = 10;
    ilqr_settings.nThreads = 4;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
    ilqr_settings.lqoc_solver_settings.num_lqoc_iterations = 1000;              // number of riccati sub-iterations
    ilqr_settings.lineSearchSettings.type = LineSearchSettings::TYPE::SIMPLE;
    ilqr_settings.lineSearchSettings.debugPrint = true;
    ilqr_settings.printSummary = true;

    /* STEP 2-B: provide an initial guess
    * iLQR requires and initial control policy; we set all control action to zero. */
    int kNUM_STEPS = ilqr_settings.computeK(timeHorizon);
    FeedbackArray<STATE_DIM, CONTROL_DIM> u0_fb(kNUM_STEPS, FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero());
    ControlVectorArray<CONTROL_DIM> u0_ff(kNUM_STEPS, ControlVector<CONTROL_DIM>::Zero());
    StateVectorArray<STATE_DIM> x_ref_init(kNUM_STEPS + 1, x0);
    NLOptConSolver<STATE_DIM, CONTROL_DIM>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

    // STEP 2-C: Create problem and solver instance
    ContinuousOptConProblem<STATE_DIM, CONTROL_DIM> optConProblem(
        timeHorizon, x0, switchedSystem, quadraticCostFunction, switchedLinearSystem);

    // Add the constraints
    optConProblem.setGeneralConstraints(switchedConstraints);

    // Setup solver with problem and solver settings
    NLOptConSolver<STATE_DIM, CONTROL_DIM> iLQR(optConProblem, ilqr_settings);

    // Add the initial guess
    iLQR.setInitialGuess(initController);

    // STEP 3: solve the optimal control problem
    iLQR.solve();

    // STEP 4: retrieve the solution
    ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> solution = iLQR.getSolution();

    // Plot results
    plotResults(solution.x_ref(), solution.uff(), solution.time());
}