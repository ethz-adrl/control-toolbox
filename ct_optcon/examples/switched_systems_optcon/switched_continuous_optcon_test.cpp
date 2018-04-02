/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include "TestSystems.h"
#include <gtest/gtest.h>

using namespace ct;
using namespace ct::core;
using namespace ct::optcon;
using std::shared_ptr;


static const size_t STATE_DIM = 2;
static const size_t CONTROL_DIM = 1;

class stateSumConstraint : public ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename ct::core::tpl::TraitSelector<double>::Trait Trait;
    typedef typename ct::core::tpl::TraitSelector<ct::core::ADCGScalar>::Trait TraitCG;
    typedef ct::optcon::ConstraintBase<STATE_DIM, CONTROL_DIM> Base;
    typedef ct::core::StateVector<STATE_DIM> state_vector_t;
    typedef ct::core::ControlVector<CONTROL_DIM> control_vector_t;

    typedef Eigen::Matrix<double, 1, STATE_DIM> Jacobian_state_t;
    typedef Eigen::Matrix<double, 1, CONTROL_DIM> Jacobian_control_t;

    //! constructor with hard-coded constraint boundaries.
    stateSumConstraint(double lb, double ub) : lb_(lb), ub_(ub)
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(lb);
        Base::ub_.setConstant(ub);
    }

    virtual ~stateSumConstraint() {}
    virtual stateSumConstraint* clone() const override { return new stateSumConstraint(lb_, ub_); }
    virtual size_t getConstraintSize() const override { return 1; }
    virtual Eigen::VectorXd evaluate(const state_vector_t& x, const control_vector_t& u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        val.template segment<1>(0) << x(0) + x(1);
        return val;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const ct::core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;

        val.template segment<1>(0) << x(0) + x(1);

        return val;
    }

private:
    double lb_;
    double ub_;
};


template <size_t STATE_DIM, size_t CONTROL_DIM>
void plotResults(const ct::core::StateVectorArray<STATE_DIM>& stateArray,
    const ct::core::ControlVectorArray<CONTROL_DIM>& controlArray,
    const ct::core::TimeArray& timeArray)
{
#ifdef PLOTTING_ENABLED
    using namespace ct::core;

    try
    {
        plot::ion();
        plot::figure();

        if (timeArray.size() != stateArray.size())
        {
            std::cout << timeArray.size() << std::endl;
            std::cout << stateArray.size() << std::endl;
            throw std::runtime_error("Cannot plot data, x and t not equal length");
        }

        std::vector<std::vector<double>> states;
        std::vector<double> time_state;
        std::vector<double> constraint;
        for (size_t k = 0; k < STATE_DIM; k++)
        {
            states.push_back(std::vector<double>());
        }

        for (size_t j = 0; j < stateArray.size(); j++)
        {
            for (size_t k = 0; k < STATE_DIM; k++)
            {
                states[k].push_back(stateArray[j](k));
            }
            time_state.push_back(timeArray[j]);
            constraint.push_back(stateArray[j](0) + stateArray[j](1));
        }

        std::vector<double> control;
        std::vector<double> time_control;
        for (size_t j = 0; j < controlArray.size(); j++)
        {
            control.push_back(controlArray[j](0));
            time_control.push_back(timeArray[j]);
        }

        for (size_t k = 0; k < STATE_DIM; k++)
        {
            plot::subplot(STATE_DIM, 1, k + 1);
            plot::plot(time_state, states[k]);
            plot::title("x(" + std::to_string(k) + ")");
        }

        plot::figure();
        plot::plot(time_state, constraint);
        plot::title("Constraint x(0) + x(1)");


        plot::figure();
        plot::plot(time_control, control);
        plot::title("Control");

        plot::show();
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
#else
    std::cout << "Plotting is disabled." << std::endl;
#endif
}

int main(int argc, char** argv)
{
    // Problem derived from Example 3 in http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1259455

    // Convenience aliases
    using MySys = TestLinearSystem;
    using MySwitchedSys = SwitchedControlledSystem<STATE_DIM, CONTROL_DIM>;
    using SystemPtr = MySwitchedSys::SystemPtr;
    using SwitchedSystems = MySwitchedSys::SwitchedSystems;
    using ConstantController = ConstantController<STATE_DIM, CONTROL_DIM>;
    using Controller = std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM>>;

    using SwitchedLinearSystem = SwitchedLinearSystem<STATE_DIM, CONTROL_DIM>;
    using SystemLinearizer = SystemLinearizer<STATE_DIM, CONTROL_DIM>;
    using LinearizerSystemPtr = SwitchedLinearSystem::LinearSystemPtr;
    using SwitchedLinearSystems = SwitchedLinearSystem::SwitchedLinearSystems;

    // == Settings =======================================================================================================
    double t_switch = 1.1624;
    double timeHorizon = 2.0;
    MySys::state_vector_t x0;
    x0 << 1.0, 1.0;

    // == Cost Function ==================================================================================================
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

    // == System Dynamics ================================================================================================
    MySys::state_matrix_t A1_continuous, A2_continuous;
    A1_continuous << 1.5, 0.0, 0.0, 1.0;
    A2_continuous << 0.5, 0.866, 0.866, -0.5;
    MySys::state_control_matrix_t B1_continuous, B2_continuous;
    B1_continuous << 1.0, 1.0;
    B2_continuous << 1.0, 1.0;
    SystemPtr sysPtr1(new MySys(A1_continuous, B1_continuous));
    SystemPtr sysPtr2(new MySys(A2_continuous, B2_continuous));
    SwitchedSystems switchedSystems = {sysPtr1, sysPtr2};

    // Setup Constant Controller
    MySys::control_vector_t u0;
    u0.setZero();
    Controller controller(new ConstantController(u0));

    // Linearization
    sysPtr1->setController(controller);
    sysPtr2->setController(controller);
    LinearizerSystemPtr linSys1(new SystemLinearizer(sysPtr1));
    LinearizerSystemPtr linSys2(new SystemLinearizer(sysPtr2));
    SwitchedLinearSystems switchedLinearSystems = {linSys1, linSys2};

    // Setup mode sequence
    ContinuousModeSequence cm_seq;
    cm_seq.addPhase(0, t_switch);                // phase 0, t in [0, t)
    cm_seq.addPhase(1, timeHorizon - t_switch);  // phase 1, t in [t, T)

    // Construct Switched Continuous System and its linearizations
    auto mySwitchedSys = std::shared_ptr<MySwitchedSys>(new MySwitchedSys(switchedSystems, cm_seq, controller));
    auto switchedLinearSystem =
        std::shared_ptr<SwitchedLinearSystem>(new SwitchedLinearSystem(switchedLinearSystems, cm_seq));

    // == Constraints ====================================================================================================
    std::shared_ptr<stateSumConstraint> phase1Constraint(new stateSumConstraint(-1e20, 7.0));
    std::shared_ptr<stateSumConstraint> phase2Constraint(new stateSumConstraint(7.0, 1e20));

    // Linearized constraints
    bool verbose = false;
    std::shared_ptr<ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>> generalConstraints_1(
        new ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>());
    generalConstraints_1->addIntermediateConstraint(phase1Constraint, verbose);
    generalConstraints_1->initialize();

    std::shared_ptr<ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>> generalConstraints_2(
        new ct::optcon::ConstraintContainerAD<STATE_DIM, CONTROL_DIM>());
    generalConstraints_2->addIntermediateConstraint(phase2Constraint, verbose);
    generalConstraints_2->initialize();

    // Switched constraints
    SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM>::SwitchedLinearConstraintContainers
        switchedConstraintContainers = {generalConstraints_1, generalConstraints_2};

    std::shared_ptr<SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM>> switchedConstraints(
        new SwitchedLinearConstraintContainer<STATE_DIM, CONTROL_DIM>(switchedConstraintContainers, cm_seq));

    switchedConstraints->initialize();

    // == Problem ========================================================================================================
    NLOptConSettings ilqr_settings;
    ilqr_settings.dt = 0.001;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 100;
    ilqr_settings.min_cost_improvement = 1e-6;
    ilqr_settings.meritFunctionRhoConstraints = 10;
    ilqr_settings.nThreads = 4;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
    ilqr_settings.lqoc_solver_settings.num_lqoc_iterations = 1000;              // number of riccati sub-iterations
    ilqr_settings.lineSearchSettings.active = true;
    ilqr_settings.lineSearchSettings.debugPrint = true;
    ilqr_settings.printSummary = true;

    // Initial guess
    int kNUM_STEPS = ilqr_settings.computeK(timeHorizon);
    FeedbackArray<STATE_DIM, CONTROL_DIM> u0_fb(kNUM_STEPS, FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero());
    ControlVectorArray<CONTROL_DIM> u0_ff(kNUM_STEPS, ControlVector<CONTROL_DIM>::Zero());
    StateVectorArray<STATE_DIM> x_ref_init(kNUM_STEPS + 1, x0);
    NLOptConSolver<STATE_DIM, CONTROL_DIM>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

    // Initialize Problem
    ContinuousOptConProblem<STATE_DIM, CONTROL_DIM> optConProblem(
        timeHorizon, x0, mySwitchedSys, quadraticCostFunction, switchedLinearSystem);
    optConProblem.setGeneralConstraints(switchedConstraints);
    NLOptConSolver<STATE_DIM, CONTROL_DIM> iLQR(optConProblem, ilqr_settings);
    iLQR.setInitialGuess(initController);

    iLQR.solve();

    // == Postprocessing =================================================================================================
    ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> solution = iLQR.getSolution();
    plotResults<STATE_DIM, CONTROL_DIM>(solution.x_ref(), solution.uff(), solution.time());
}