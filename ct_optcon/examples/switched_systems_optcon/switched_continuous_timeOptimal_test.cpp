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


static const size_t STATE_DIM = 3;
static const size_t CONTROL_DIM = 2;

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

  try {
    plot::ion();


    if (timeArray.size() != stateArray.size()) {
      std::cout << timeArray.size() << std::endl;
      std::cout << stateArray.size() << std::endl;
      throw std::runtime_error("Cannot plot data, x and t not equal length");
    }

    std::vector<std::vector<double>> states;
    std::vector<double> time_state;
    std::vector<double> constraint;
    for (size_t k = 0; k < STATE_DIM; k++){
      states.push_back(std::vector<double>());
    }

    for (size_t j = 0; j < stateArray.size(); j++)
    {
      for (size_t k = 0; k < STATE_DIM; k++){
        states[k].push_back(stateArray[j](k));
      }
      time_state.push_back(timeArray[j]);
      constraint.push_back(stateArray[j](0) + stateArray[j](1));
    }

    std::vector<std::vector<double>> control;
    std::vector<double> time_control;
    for (size_t k = 0; k < CONTROL_DIM; k++){
      control.push_back(std::vector<double>());
    }


    for (size_t j = 0; j < controlArray.size(); j++)
    {
      for (size_t k = 0; k < CONTROL_DIM; k++){
        control[k].push_back(controlArray[j](k));
      }
      time_control.push_back(timeArray[j]);
    }

    plot::figure();
    for (size_t k = 0; k < STATE_DIM; k++) {
      plot::subplot(STATE_DIM, 1, k+1);
      plot::plot(time_state, states[k]);
      plot::title("x(" + std::to_string(k) + ")");
    }

    plot::figure();
    for (size_t k = 0; k < CONTROL_DIM; k++) {
      plot::subplot(CONTROL_DIM, 1, k+1);
      plot::plot(time_control, control[k]);
      plot::title("u(" + std::to_string(k) + ")");
    }

    plot::figure();
    plot::plot(time_state, constraint);
    plot::title("Constraint x(0) + x(1)");

    plot::show();
  } catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
#else
  std::cout << "Plotting is disabled." << std::endl;
#endif
}

class CostFunctionTimeOptimal : public CostFunctionAnalytical<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, double> state_vector_t;
    typedef core::ControlVector<CONTROL_DIM, double> control_vector_t;

    CostFunctionTimeOptimal() {
      rho_ = 0.1;
    };

    CostFunctionTimeOptimal(const CostFunctionTimeOptimal& arg) {};

    CostFunctionTimeOptimal* clone () const {return new CostFunctionTimeOptimal(*this);};

    ~CostFunctionTimeOptimal() {};

    double evaluateIntermediate() override {
      return 0.5 * this->u_(1) * this->u_(0) * this->u_(0) + 0.5 * rho_ * (this->u_(1) - 1.0) *(this->u_(1) - 1.0) ;
    };

    double evaluateTerminal() override {
      return 0.5 * (this->x_(0) - 10.0) * (this->x_(0) - 10.0) +
          0.5 * (this->x_(1) - 6.0) * (this->x_(1) - 6.0);
    };

    state_vector_t stateDerivativeIntermediate() override
    {
      state_vector_t qv;
      qv.setZero();
      return qv;
    };

    state_vector_t stateDerivativeTerminal() override
    {
      state_vector_t qv;
      qv << (this->x_(0) - 10.0), (this->x_(1) - 6.0), 0.0;
      return qv;
    };

    state_matrix_t stateSecondDerivativeIntermediate() override
    {
      state_matrix_t Q;
      Q.setZero();
      return Q;
    };

    state_matrix_t stateSecondDerivativeTerminal() override{
      state_matrix_t Q;
      Q << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 0.0;
      return Q;
    };

    control_vector_t controlDerivativeIntermediate() override {
      control_vector_t rv;
      rv << this->u_(1) * this->u_(0),
          0.5 * this->u_(0) * this->u_(0) + rho_ * (this->u_(1) - 1.0);
      return rv;
    };

    control_vector_t controlDerivativeTerminal() override {
      control_vector_t rv;
      rv.setZero();
      return rv;
    };

    control_matrix_t controlSecondDerivativeIntermediate() override {
      control_matrix_t R;
      R << this->u_(1), this->u_(0),
           this->u_(0), rho_;
      return R;
    };

    control_matrix_t controlSecondDerivativeTerminal() override {
      control_matrix_t R;
      R.setZero();
      return R;
    };

    control_state_matrix_t stateControlDerivativeIntermediate() override {
      control_state_matrix_t P;
      P.setZero();
      return P;
    };

    control_state_matrix_t stateControlDerivativeTerminal() override {
      control_state_matrix_t P;
      P.setZero();
      return P;
    };

private:
    double rho_;

};

int main(int argc, char **argv)
{
  // Problem derived from Example 3 in http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1259455

  // Convenience aliases
  using MySys = TestTimeParameterizedLinearSystem;
  typedef CppAD::AD<CppAD::cg::CG<double>> Scalar;
  using MySysAD = ct::core::tpl::TestTimeParameterizedLinearSystem<Scalar>;
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
  bool verbose = false;
  double t_switch = 1.0;
  double timeHorizon = 2.0;
  MySys::state_vector_t x0;
  x0 << 1.0, 1.0, 0.0;

  // == Cost Function ==================================================================================================
  std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFunction(new CostFunctionTimeOptimal());

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

  // AD
  MySysAD::state_matrix_t A1_AD, A2_AD;
  A1_AD << Scalar(1.5), Scalar(0.0), Scalar(0.0), Scalar(1.0);
  A2_AD << Scalar(0.5), Scalar(0.866), Scalar(0.866), Scalar(-0.5);
  MySysAD::state_control_matrix_t B1_AD, B2_AD;
  B1_AD << Scalar(1.0), Scalar(1.0);
  B2_AD << Scalar(1.0), Scalar(1.0);
  std::shared_ptr<MySysAD> sysPtrAD1( new MySysAD(A1_AD, B1_AD));
  std::shared_ptr<MySysAD> sysPtrAD2( new MySysAD(A2_AD, B2_AD));

  // Setup Constant Controller
  MySys::control_vector_t u0;
  u0 << 0.0, 1.0;
  Controller controller(new ConstantController(u0));

  // Linearization
  std::shared_ptr<ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>> linSys1(new ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>(sysPtrAD1));
  std::shared_ptr<ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>> linSys2(new ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>(sysPtrAD2));
  linSys1->compileJIT("linSys1_JIT");
  linSys2->compileJIT("linSys2_JIT");
  SwitchedLinearSystems switchedLinearSystems = {linSys1, linSys2};

  // Setup mode sequence
  ContinuousModeSequence cm_seq;
  cm_seq.addPhase(0, t_switch);  // phase 0, t in [0, t)
  cm_seq.addPhase(1, timeHorizon - t_switch);  // phase 1, t in [t, T)

  // Construct Switched Continuous System and its linearizations
  auto mySwitchedSys = std::shared_ptr<MySwitchedSys>(new MySwitchedSys(switchedSystems, cm_seq, controller));
  auto switchedLinearSystem = std::shared_ptr<SwitchedLinearSystem>(new SwitchedLinearSystem(switchedLinearSystems, cm_seq));

  // == Constraints ====================================================================================================

  // -- Box constraints --
  std::shared_ptr<ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>> boxConstraints(
      new ct::optcon::ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>());

  // Intermediate constraint terms
  Eigen::VectorXi sp_control(CONTROL_DIM);
  Eigen::VectorXd u_lb(1);
  Eigen::VectorXd u_ub(1);
  sp_control << 0, 1;
  u_lb.setConstant(0.5);
  u_ub.setConstant(2.0);

  std::shared_ptr<ControlInputConstraint<STATE_DIM, CONTROL_DIM>> controlConstraint(
      new ControlInputConstraint<STATE_DIM, CONTROL_DIM>(u_lb, u_ub, sp_control));

  // Terminal constraint terms
  Eigen::VectorXi sp_state_final(STATE_DIM);
  Eigen::VectorXd x_lb(1);
  Eigen::VectorXd x_ub(1);
  sp_state_final << 0, 0, 1;
  x_lb.setConstant(timeHorizon);
  x_ub.setConstant(timeHorizon);

  std::shared_ptr<StateConstraint<STATE_DIM, CONTROL_DIM>> stateConstraint(
      new StateConstraint<STATE_DIM, CONTROL_DIM>(x_lb, x_ub, sp_state_final));

  // add and initialize constraint terms
  boxConstraints->addIntermediateConstraint(controlConstraint, verbose);
  boxConstraints->addTerminalConstraint(stateConstraint, verbose);
  boxConstraints->initialize();

  // -- General constraints --
  std::shared_ptr<stateSumConstraint> phase1Constraint(new stateSumConstraint(-1e20, 7.0));
  std::shared_ptr<stateSumConstraint> phase2Constraint(new stateSumConstraint(7.0, 1e20));

  // Linearized constraints
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
  ilqr_settings.dt = 0.01;  // the control discretization in [sec]
  ilqr_settings.integrator = ct::core::IntegrationType::RK4;
  ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
  ilqr_settings.max_iterations = 10;
  ilqr_settings.min_cost_improvement = 1e-6;
  ilqr_settings.meritFunctionRhoConstraints = 10;
  ilqr_settings.nThreads = 4;
  ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
  ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;    // solve LQ-problems using HPIPM
  ilqr_settings.lqoc_solver_settings.num_lqoc_iterations = 1000;                // number of riccati sub-iterations
  ilqr_settings.lineSearchSettings.active = true;
  ilqr_settings.lineSearchSettings.adaptive = false;
  ilqr_settings.lineSearchSettings.maxIterations = 10;
  ilqr_settings.lineSearchSettings.debugPrint = true;
  ilqr_settings.printSummary = true;

  // Initial guess
  int kNUM_STEPS = ilqr_settings.computeK(timeHorizon);
  FeedbackArray<STATE_DIM, CONTROL_DIM> u0_fb(kNUM_STEPS, FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero());
  ControlVectorArray<CONTROL_DIM> u0_ff(kNUM_STEPS, u0);
  StateVectorArray<STATE_DIM> x_ref_init(kNUM_STEPS + 1, x0);
  NLOptConSolver<STATE_DIM, CONTROL_DIM>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

  // Initialize Problem
  ContinuousOptConProblem<STATE_DIM, CONTROL_DIM> optConProblem(timeHorizon,
                                                                x0,
                                                                mySwitchedSys,
                                                                costFunction,
                                                                switchedLinearSystem);
  optConProblem.setBoxConstraints(boxConstraints);
  optConProblem.setGeneralConstraints(switchedConstraints);
  NLOptConSolver<STATE_DIM, CONTROL_DIM> iLQR(optConProblem, ilqr_settings);
  iLQR.setInitialGuess(initController);


  iLQR.solve();

    // == Postprocessing =================================================================================================
  ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> solution = iLQR.getSolution();

  plotResults<STATE_DIM, CONTROL_DIM>(solution.x_ref(), solution.uff(), solution.time());

}