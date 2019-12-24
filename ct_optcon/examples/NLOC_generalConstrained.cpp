/*!
 * \example NLOC_generalConstrained.cpp
 *
 * This example shows how to use general constraints alongside NLOC and requires HPIPM to be installed
 * The unconstrained Riccati backward-pass is replaced by a high-performance interior-point
 * constrained linear-quadratic Optimal Control solver.
 *
 */

#include <ct/optcon/optcon.h>
#include "exampleDir.h"
#include "plotResultsOscillator.h"

using namespace ct::core;
using namespace ct::optcon;


/*get the state and control input dimension of the oscillator. Since we're dealing with a simple oscillator,
 the state and control dimensions will be state_dim = 2, and control_dim = 1. */
static const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
static const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;

/*!
 * @brief A simple 1d constraint term.
 *
 * This term implements the general inequality constraints
 * \f$ d_{lb} \leq u \cdot p^2 \leq d_{ub} \f$
 * where \f$ p \f$ denotes the position of the oscillator mass.
 *
 * This constraint can be thought of a position-varying bound on the control input.
 * At large oscillator deflections, the control bounds shrink
 */
class ConstraintTerm1D : public ct::optcon::ConstraintBase<state_dim, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ct::optcon::ConstraintBase<state_dim, control_dim> Base;
    typedef ct::core::StateVector<state_dim> state_vector_t;
    typedef ct::core::ControlVector<control_dim> control_vector_t;

    //! constructor with hard-coded constraint boundaries.
    ConstraintTerm1D()
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(-1.0);
        Base::ub_.setConstant(1.0);
    }

    virtual ~ConstraintTerm1D() {}
    virtual ConstraintTerm1D* clone() const override { return new ConstraintTerm1D(); }
    virtual size_t getConstraintSize() const override { return 1; }
    virtual Eigen::VectorXd evaluate(const state_vector_t& x, const control_vector_t& u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        val.template segment<1>(0) << u(0) * x(0) * x(0);
        return val;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<state_dim, ct::core::ADCGScalar>& x,
        const ct::core::ControlVector<control_dim, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;
        val.template segment<1>(0) << u(0) * x(0) * x(0);
        return val;
    }
};


int main(int argc, char** argv)
{
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


    /* STEP 1-D: set up the general constraints */
    // constraint terms
    std::shared_ptr<ConstraintTerm1D> pathConstraintTerm(new ConstraintTerm1D());

    // create constraint container
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> generalConstraints(
        new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());


    // add and initialize constraint terms
    generalConstraints->addIntermediateConstraint(pathConstraintTerm, verbose);
    generalConstraints->addTerminalConstraint(pathConstraintTerm, verbose);
    generalConstraints->initialize();


    /* STEP 1-E: initialization with initial state and desired time horizon */

    StateVector<state_dim> x0;
    x0.setZero();

    ct::core::Time timeHorizon = 3.0;  // and a final time horizon in [sec]


    // STEP 1-E: create and initialize an "optimal control problem"
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x0, oscillatorDynamics, costFunction, adLinearizer);

    // add the box constraints to the optimal control problem
    optConProblem.setGeneralConstraints(generalConstraints);


    /* STEP 2: set up a nonlinear optimal control solver. */

    /* STEP 2-A: Create the settings.
	 * the type of solver, and most parameters, like number of shooting intervals, etc.,
	 * can be chosen using the following settings struct. Let's use Gauss-Newton
	 * Multiple Shooting for this example. In the following, we
	 * modify only a few settings, for more detail, check out the NLOptConSettings class. */
    NLOptConSettings nloc_settings;
    nloc_settings.load(ct::optcon::exampleDir + "/nlocSolver.info", true, "ilqr");
    nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM

    /* STEP 2-B: provide an initial guess */
    // calculate the number of time steps K
    size_t K = nloc_settings.computeK(timeHorizon);

    /* design trivial initial controller for NLOC. Note that in this simple example,
	 * we can simply use zero feedforward with zero feedback gains around the initial position.
	 * In more complex examples, a more elaborate initial guess may be required.*/
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);


    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver<state_dim, control_dim> nloc(optConProblem, nloc_settings);

    // set the initial guess
    nloc.setInitialGuess(initController);


    // STEP 3: solve the optimal control problem
    nloc.solve();

    // STEP 4: retrieve the solution
    ct::core::StateFeedbackController<state_dim, control_dim> solution = nloc.getSolution();

    // plot the output
    plotResultsOscillator<state_dim, control_dim>(solution.x_ref(), solution.uff(), solution.time());
}
