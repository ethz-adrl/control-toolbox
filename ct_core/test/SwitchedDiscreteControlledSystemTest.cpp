/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "system/TestDiscreteNonlinearSystem.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;
using std::shared_ptr;

TEST(SwitchedDiscreteControlledSystemTest, SwitchedDiscreteControlledSystem)
{
  // Convenience aliases
  using MySys = TestDiscreteNonlinearSystem;
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
  SystemPtr sysPtr1(new MySys(1.0));
  SystemPtr sysPtr2(new MySys(2.0));
  SwitchedSystems switchedSystems = {sysPtr1, sysPtr2};

  // Setup mode sequence
  DiscreteModeSequence dm_seq;
  dm_seq.addPhase(0, 2);  // phase 0, t in [0, 2)
  dm_seq.addPhase(1, 3);  // phase 1, t in [2, 5)
  dm_seq.addPhase(0, 1);  // phase 2, t in [5, 6)

  // Setup Constant Controller
  MySys::control_vector_t u;
  u[0] = 1.0;
  Controller controller(new ConstantController(u));

  // Construct Switched Discrete System
  MySwitchedSys mySwitchedSys(switchedSystems, dm_seq, controller);

  // Forward Integrate
  MySys::state_vector_t x, x_next;
  x.setZero();
  x[0] = 1.0;
  std::vector<double> intermediate_solutions = {1.0, 2.0, 4.0, 12.0, 36.0, 108.0, 216.0};
  for (int t=dm_seq.getStartTimeFromIdx(0); t<dm_seq.getTotalDuration(); t++){
    ASSERT_DOUBLE_EQ(x[0], intermediate_solutions[t]);
    mySwitchedSys.propagateDynamics(x, t, x_next);
    x = x_next;
    ASSERT_DOUBLE_EQ(x[0], intermediate_solutions[t+1]);
  }

  // Linearization
  sysPtr1->setController(controller);
  sysPtr2->setController(controller);
  LinearizerSystemPtr linSys1(new DiscreteSystemLinearizer(sysPtr1));
  LinearizerSystemPtr linSys2(new DiscreteSystemLinearizer(sysPtr2));

  // Switched Linearization
  SwitchedLinearSystems switchedLinearSystems = {linSys1, linSys2};
  SwitchedDiscreteLinearSystem switchedDiscreteLinearSystem(switchedLinearSystems, dm_seq);

  // Test linearizations
  SwitchedDiscreteLinearSystem::state_matrix_t A;
  SwitchedDiscreteLinearSystem::state_control_matrix_t B;
  x.setZero();
  x[0] = 1.0;
  std::vector<double> B0_solutions = {1.0, 1.0, 2.0, 2.0, 2.0, 1.0, 1.0};
  for (int t=dm_seq.getStartTimeFromIdx(0); t<dm_seq.getTotalDuration(); t++){
    switchedDiscreteLinearSystem.getAandB(x, u, t, A, B);
    ASSERT_DOUBLE_EQ(B[0], B0_solutions[t]);
  }

}


/*!
 *  \example SwitchingDiscreteControlledSystemTest.cpp
 *
 *  Test basic functionality of switching discrete controlled systems
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}