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

int main(int argc, char **argv)
{
  const bool VERBOSE = false;

  // Convenience aliases
  using MySys = TestDiscreteLinearSystem;
  using MySwitchedSys = SwitchedDiscreteControlledSystem<MySys::STATE_DIM, MySys::CONTROL_DIM>;
  using SystemPtr = MySwitchedSys::SystemPtr;
  using SwitchedSystems = MySwitchedSys::SwitchedSystems;
  using ConstantController = ConstantController<MySys::STATE_DIM, MySys::CONTROL_DIM>;
  using Controller = std::shared_ptr<DiscreteController<MySys::STATE_DIM, MySys::CONTROL_DIM>>;

  using SwitchedDiscreteLinearSystem = SwitchedDiscreteLinearSystem<MySys::STATE_DIM, MySys::CONTROL_DIM>;
  using DiscreteSystemLinearizer = DiscreteSystemLinearizer<MySys::STATE_DIM, MySys::CONTROL_DIM>;
  using LinearizerSystemPtr = SwitchedDiscreteLinearSystem::LinearSystemPtr;
  using SwitchedLinearSystems = SwitchedDiscreteLinearSystem::SwitchedLinearSystems;

  // Setup systems
  // Example 3 in http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1259455
  int N1 = 50;
  int N2 = 50;
  double dt1 = 1.1621 / ((double) N1);
  double dt2 = (2.0 - 1.1621) / ((double) N2);
  MySys::state_matrix_t A1_continuous, A2_continuous;
  A1_continuous << 1.5, 0.0, 0.0, 1.0;
  A2_continuous << 0.5, 0.866, 0.866, -0.5;
  MySys::state_control_matrix_t B1_continuous, B2_continuous;
  B1_continuous << 1.0, 1.0;
  B2_continuous << 1.0, 1.0;
  MySys::state_matrix_t A1_discrete = (MySys::state_matrix_t::Identity() + A1_continuous * dt1);
  MySys::state_matrix_t A2_discrete = (MySys::state_matrix_t::Identity() + A2_continuous * dt2);
  MySys::state_control_matrix_t B1_discrete = B1_continuous * dt1;
  MySys::state_control_matrix_t B2_discrete = B2_continuous * dt2;
  SystemPtr sysPtr1(new MySys(A1_discrete, B1_discrete));
  SystemPtr sysPtr2(new MySys(A2_discrete, B2_discrete));
  SwitchedSystems switchedSystems = {sysPtr1, sysPtr2};

  // Setup mode sequence
  DiscreteModeSequence dm_seq;
  dm_seq.addPhase(0, N1);  // phase 0, t in [0, N1)
  dm_seq.addPhase(1, N2);  // phase 1, t in [N1, N1+N2)

  // Setup Constant Controller
  MySys::control_vector_t u0;
  u0.setZero();
  Controller controller(new ConstantController(u0));

  // Construct Switched Discrete System
  MySwitchedSys mySwitchedSys(switchedSystems, dm_seq, controller);

  // Linearization
  sysPtr1->setController(controller);
  sysPtr2->setController(controller);
  LinearizerSystemPtr linSys1(new DiscreteSystemLinearizer(sysPtr1));
  LinearizerSystemPtr linSys2(new DiscreteSystemLinearizer(sysPtr2));

  // Switched Linearization
  SwitchedLinearSystems switchedLinearSystems = {linSys1, linSys2};
  SwitchedDiscreteLinearSystem switchedDiscreteLinearSystem(switchedLinearSystems, dm_seq);

  // Cost function
  Eigen::Matrix<double, 2, 2> Q;
  Q << 0.0, 0, 0, 0.0;
  Eigen::Matrix<double, 1, 1> R;
  R[0] = 1.0;
  Eigen::Matrix<double, 2, 1> x_final;
  x_final << 10.0, 6.0;
  Eigen::Matrix<double, 2, 1> x_nominal;
  x_nominal.setZero();
  Eigen::Matrix<double, 1, 1> u_nominal;
  u_nominal.setZero();
  Eigen::Matrix<double, 2, 2> Q_final;
  Q_final << 1.0, 0, 0, 1.0;

  std::shared_ptr<CostFunctionQuadratic<2, 1>> quadraticCostFunction(
      new CostFunctionQuadraticSimple<2, 1>(Q, R, x_nominal, u_nominal, x_final, Q_final));

  // Initialize Problem
  const size_t N = dm_seq.getTotalDuration();
  std::shared_ptr<LQOCProblem<MySys::STATE_DIM, MySys::CONTROL_DIM>> lqocProblem(new LQOCProblem<MySys::STATE_DIM, MySys::CONTROL_DIM>(N));

  SwitchedDiscreteLinearSystem::state_matrix_t A;
  SwitchedDiscreteLinearSystem::state_control_matrix_t B;
  MySys::state_vector_t x0;
  x0 << 1.0, 1.0;
  lqocProblem->setZero();
  // Set Intermediate properties
  for (int t=dm_seq.getStartTimeFromIdx(0); t<dm_seq.getTotalDuration(); t++){
    // Linearization state action
    lqocProblem->x_[t] = x0;
    lqocProblem->u_[t] = u0;

    // Dynamics
    switchedDiscreteLinearSystem.getAandB(x0, u0, t, A, B);
    lqocProblem->A_[t] = A;
    lqocProblem->B_[t] = B;
    lqocProblem->b_[t] = A*x0 + B*u0 - x0; // To achieve relative formulation: d_x(t+1) = A(t)*d_x(t) + B(t)*d_u(t) + b(t)

    // Intermediate Costs
    quadraticCostFunction->setCurrentStateAndControl(x0, u0);

    double dt = t < N1 ? dt1 : dt2;
    lqocProblem->qv_[t] = quadraticCostFunction->stateDerivativeIntermediate() * dt;
    lqocProblem->Q_[t] = quadraticCostFunction->stateSecondDerivativeIntermediate() * dt;
    lqocProblem->P_[t] = quadraticCostFunction->stateControlDerivativeIntermediate() * dt;
    lqocProblem->rv_[t] = quadraticCostFunction->controlDerivativeIntermediate() * dt;
    lqocProblem->R_[t] = quadraticCostFunction->controlSecondDerivativeIntermediate() * dt;

    // Constraints
    // Manually implement switching surface x1 + x2 - 7 = 0 @ t = N1
    if (t==N1) {
      lqocProblem->ng_[t] = 1;
      lqocProblem->d_lb_[t].resize(1, 1);
      lqocProblem->d_lb_[t] << 7.0;
      lqocProblem->d_ub_[t].resize(1, 1);
      lqocProblem->d_ub_[t] << 7.0;
      lqocProblem->C_[t].resize(1, MySys::STATE_DIM);
      lqocProblem->C_[t] << 1.0, 1.0;
      lqocProblem->D_[t].resize(1, MySys::CONTROL_DIM);
      lqocProblem->D_[t].setZero();
    }

    if (VERBOSE) {
      std::cout << "Setting up t = " << t << std::endl;
      std::cout << "A: " << std::endl << lqocProblem->A_[t] << std::endl;
      std::cout << "B: " << std::endl << lqocProblem->B_[t] << std::endl;
      std::cout << "b: " << std::endl << lqocProblem->b_[t] << std::endl;
      std::cout << "Q: " << std::endl << lqocProblem->Q_[t] << std::endl;
      std::cout << "R: " << std::endl << lqocProblem->R_[t] << std::endl;
    }
  }
  // Terminal Stage
  lqocProblem->x_[dm_seq.getTotalDuration()] = x0;
  lqocProblem->qv_[dm_seq.getTotalDuration()] = quadraticCostFunction->stateDerivativeTerminal();
  lqocProblem->Q_[dm_seq.getTotalDuration()] = quadraticCostFunction->stateSecondDerivativeTerminal();

  if (VERBOSE) {
    std::cout << "Final stage" << std::endl;
    std::cout << "q: " << std::endl << lqocProblem->qv_.back().transpose() << std::endl;
    std::cout << "Q: " << std::endl << lqocProblem->Q_.back() << std::endl;
  }

  // Settings
  lqocProblem->hasBoxConstraints_ = false;
  lqocProblem->hasGenConstraints_ = true;

  // SOLVE
    std::shared_ptr<LQOCSolver<MySys::STATE_DIM, MySys::CONTROL_DIM>> solver(new HPIPMInterface<MySys::STATE_DIM, MySys::CONTROL_DIM>);
  solver->setProblem(lqocProblem);
  solver->solve();

  // Solution
  auto xSol = solver->getSolutionState();
  auto uSol = solver->getSolutionControl();
  for (int t=dm_seq.getStartTimeFromIdx(0); t<dm_seq.getTotalDuration(); t++){
    if (t==N1){
      std::cout << "!! SWITCH !!" << std::endl;
    }
    std::cout << "t: " << t
              << "\tx: " << xSol[t].transpose()
              << "\tu: " << uSol[t].transpose()
              << std::endl;
  }
  std::cout << "t: " << dm_seq.getTotalDuration()
            << "\tx: " << xSol[dm_seq.getTotalDuration()].transpose()
            << std::endl;

}