
#include <ct/optcon/optcon.h>
#include "nloc_test_dir.h"
#include <gtest/gtest.h>

using namespace ct::core;
using namespace ct::optcon;

const bool verbose = false;

const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;

StateVector<state_dim> x0 = StateVector<state_dim>::Zero();
ct::core::Time timeHorizon = 3.0;

// system dynamics
const double w_n = 0.1;
const double zeta = 5.0;
std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
    new ct::core::SecondOrderSystem(w_n, zeta));

// linearized system
std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> linearizer(
    new ct::core::SystemLinearizer<state_dim, control_dim>(oscillatorDynamics));

std::string nloc_test_dir = std::string(NLOC_TEST_DIR);


void compareSolutions(const ct::core::StateFeedbackController<state_dim, control_dim>& sol1,
    const ct::core::StateFeedbackController<state_dim, control_dim>& sol2)
{
    auto sol1_x = sol1.x_ref();
    auto sol2_x = sol2.x_ref();
    auto sol1_u = sol1.uff();
    auto sol2_u = sol2.uff();

    for (size_t j = 0; j < sol1_u.size(); j++)
    {
        for (size_t n = 0; n < control_dim; n++)
        {
            ASSERT_NEAR(sol1_u[j](n), sol2_u[j](n), 1e-3);
        }
    }

    for (size_t j = 0; j < sol1_x.size(); j++)
    {
        for (size_t n = 0; n < state_dim; n++)
        {
            ASSERT_NEAR(sol1_x[j](n), sol2_x[j](n), 1e-3);
        }
    }
}


class ControlInputGenConstraint : public ct::optcon::ConstraintBase<state_dim, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ct::optcon::ConstraintBase<state_dim, control_dim> Base;
    typedef ct::core::StateVector<state_dim> state_vector_t;
    typedef ct::core::ControlVector<control_dim> control_vector_t;

    //! constructor with hard-coded constraint boundaries.
    ControlInputGenConstraint()
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(-0.5);
        Base::ub_.setConstant(0.5);
    }

    virtual ~ControlInputGenConstraint() {}
    virtual ControlInputGenConstraint* clone() const override { return new ControlInputGenConstraint(); }
    virtual size_t getConstraintSize() const override { return 1; }
    virtual Eigen::VectorXd evaluate(const state_vector_t& x, const control_vector_t& u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        val.template segment<1>(0) << u(0);
        return val;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<state_dim, ct::core::ADCGScalar>& x,
        const ct::core::ControlVector<control_dim, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;
        val.template segment<1>(0) << u(0);
        return val;
    }
};


class StateGenConstraint : public ct::optcon::ConstraintBase<state_dim, control_dim>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ct::optcon::ConstraintBase<state_dim, control_dim> Base;
    typedef ct::core::StateVector<state_dim> state_vector_t;
    typedef ct::core::ControlVector<control_dim> control_vector_t;

    //! constructor with hard-coded constraint boundaries.
    StateGenConstraint()
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_(0) = -0.2;
        Base::ub_ = -Base::lb_;
    }

    virtual ~StateGenConstraint() {}
    virtual StateGenConstraint* clone() const override { return new StateGenConstraint(); }
    virtual size_t getConstraintSize() const override { return 1; }
    virtual Eigen::VectorXd evaluate(const state_vector_t& x, const control_vector_t& u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        val(0, 0) = x(1);
        return val;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<state_dim, ct::core::ADCGScalar>& x,
        const ct::core::ControlVector<control_dim, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;
        val(0, 0) = x(1);
        return val;
    }
};


// convenience function for generating an NLOC solver
NLOptConSolver<state_dim, control_dim> generateSolver(ContinuousOptConProblem<state_dim, control_dim> ocp)
{
    NLOptConSettings nloc_settings;
    nloc_settings.dt = 0.01;  // the control discretization in [sec]
    nloc_settings.integrator = ct::core::IntegrationType::EULERCT;
    nloc_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    nloc_settings.max_iterations = 10;
    nloc_settings.nThreads = 1;
    nloc_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
    nloc_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
    nloc_settings.lqoc_solver_settings.num_lqoc_iterations = 100;               // number of riccati sub-iterations
    nloc_settings.printSummary = false;
    nloc_settings.lineSearchSettings.type = LineSearchSettings::TYPE::NONE;

    // init controller
    size_t K = nloc_settings.computeK(timeHorizon);
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

    NLOptConSolver<state_dim, control_dim> nloc(ocp, nloc_settings);

    nloc.setInitialGuess(initController);

    return nloc;
}


// convenience function for generating an unconstrained OCP
ContinuousOptConProblem<state_dim, control_dim> generateUnconstrainedOCP()
{
    // cost terms and cost function
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    intermediateCost->loadConfigFile(nloc_test_dir + "/constrained/nlocCost.info", "intermediateCost", verbose);
    finalCost->loadConfigFile(nloc_test_dir + "/constrained/nlocCost.info", "finalCost", verbose);

    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);
    costFunction->addFinalTerm(finalCost);

    return ContinuousOptConProblem<state_dim, control_dim>(
        timeHorizon, x0, oscillatorDynamics, costFunction, linearizer);
}


/*
 * This test compares pure control box constraints against the equivalent implementation as general constraints
 */
TEST(Constrained_NLOC_Test, comparePureControlConstraints)
{
    // OPTION 1 - create constraint container for box constraints
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> boxConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // create a box constraint on the control input
    Eigen::VectorXi sp_control(control_dim);
    sp_control << 1;
    Eigen::VectorXd u_lb(control_dim);
    Eigen::VectorXd u_ub(control_dim);
    u_lb.setConstant(-0.5);
    u_ub = -u_lb;

    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlConstraint(
        new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub, sp_control));
    controlConstraint->setName("ControlInputConstraint");

    boxConstraints->addIntermediateConstraint(controlConstraint, verbose);
    boxConstraints->initialize();

    auto optConProblem_box = generateUnconstrainedOCP();
    optConProblem_box.setInputBoxConstraints(boxConstraints);


    // OPTION 2 - create constraint container for "general" constraints, same constraint objective
    std::shared_ptr<ControlInputGenConstraint> pathConstraintTerm(new ControlInputGenConstraint());

    // create constraint container
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> generalConstraints(
        new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());

    generalConstraints->addIntermediateConstraint(pathConstraintTerm, verbose);
    generalConstraints->initialize();

    auto optConProblem_gen = generateUnconstrainedOCP();
    optConProblem_gen.setGeneralConstraints(generalConstraints);

    // create solvers and solve
    auto nloc_box = generateSolver(optConProblem_box);
    auto nloc_gen = generateSolver(optConProblem_gen);

    nloc_box.solve();
    nloc_gen.solve();

    ct::core::StateFeedbackController<state_dim, control_dim> solution_box = nloc_box.getSolution();
    ct::core::StateFeedbackController<state_dim, control_dim> solution_gen = nloc_gen.getSolution();

    compareSolutions(solution_box, solution_gen);
}


/*
 * This test compares pure state box constraints against the equivalent implementation as general constraints
 */
TEST(Constrained_NLOC_Test, comparePureStateConstraints)
{
    // OPTION 1 - create constraint container for box constraints
    std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> boxConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // create a box constraint on the state vector
    Eigen::VectorXi sp_state(state_dim);
    sp_state << 0, 1;
    Eigen::VectorXd x_lb(1);
    Eigen::VectorXd x_ub(1);
    x_lb.setConstant(-0.2);
    x_ub = -x_lb;

    std::shared_ptr<StateConstraint<state_dim, control_dim>> stateConstraint(
        new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sp_state));
    stateConstraint->setName("StateConstraint");

    boxConstraints->addIntermediateConstraint(stateConstraint, verbose);
    boxConstraints->initialize();

    auto optConProblem_box = generateUnconstrainedOCP();
    optConProblem_box.setStateBoxConstraints(boxConstraints);


    // OPTION 2 - create constraint container for "general" constraints, same constraint objective
    std::shared_ptr<StateGenConstraint> pathConstraintTerm(new StateGenConstraint());

    // create constraint container
    std::shared_ptr<ct::optcon::ConstraintContainerAD<state_dim, control_dim>> generalConstraints(
        new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());

    generalConstraints->addIntermediateConstraint(pathConstraintTerm, verbose);
    generalConstraints->addTerminalConstraint(pathConstraintTerm, verbose);
    generalConstraints->initialize();

    std::cout << generalConstraints->jacobianInputIntermediate() << std::endl;
    std::cout << generalConstraints->jacobianStateIntermediate() << std::endl;

    auto optConProblem_gen = generateUnconstrainedOCP();
    optConProblem_gen.setGeneralConstraints(generalConstraints);

    // create solvers and solve
    auto nloc_box = generateSolver(optConProblem_box);
    auto nloc_gen = generateSolver(optConProblem_gen);

    nloc_box.solve();
    nloc_gen.solve();

    ct::core::StateFeedbackController<state_dim, control_dim> solution_box = nloc_box.getSolution();
    ct::core::StateFeedbackController<state_dim, control_dim> solution_gen = nloc_gen.getSolution();

    compareSolutions(solution_box, solution_gen);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
