/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "../system/TestNonlinearSystem.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;
using std::shared_ptr;

TEST(SwitchedControlledSystemTest, SwitchedControlledSystem_continuous_time)
{
    const bool VERBOSE = false;

    // Convenience aliases
    using System = TestNonlinearSystem<CONTINUOUS_TIME>;
    using State_t = EuclideanState<System::STATE_DIM>;
    using SwitchedSystem = SwitchedControlledSystem<State_t, System::CONTROL_DIM, CONTINUOUS_TIME>;
    using SystemPtr = SwitchedSystem::SystemPtr;
    using SwitchedSystems = SwitchedSystem::SwitchedSystems;
    using ConstantController = ConstantController<State_t, System::CONTROL_DIM, CONTINUOUS_TIME>;
    using Controller = std::shared_ptr<Controller<State_t, System::CONTROL_DIM, CONTINUOUS_TIME>>;

    using SwitchedLinearSystem = SwitchedLinearSystem<State_t, System::CONTROL_DIM, CONTINUOUS_TIME>;
    using SystemLinearizer = SystemLinearizer<State_t, System::CONTROL_DIM, CONTINUOUS_TIME>;
    using LinearizerSystemPtr = SwitchedLinearSystem::LinearSystemPtr;
    using SwitchedLinearSystems = SwitchedLinearSystem::SwitchedLinearSystems;

    // Setup systems
    SystemPtr sysPtr1(new System(0.0));
    SystemPtr sysPtr2(new System(1.0));
    SwitchedSystems switchedSystems;
    switchedSystems.push_back(sysPtr1);
    switchedSystems.push_back(sysPtr2);

    // Setup mode sequence
    ContinuousModeSequence cm_seq;
    cm_seq.addPhase(0, 1.5);  // phase 0, t in [0.0, 1.5)
    cm_seq.addPhase(1, 2.0);  // phase 1, t in [1.5, 3.5)
    cm_seq.addPhase(0, 0.5);  // phase 2, t in [3.5, 3.0)

    // Setup Constant Controller
    System::control_vector_t u;
    u[0] = 1.0;
    Controller controller(new ConstantController(u));

    // Construct Switched System
    std::shared_ptr<SwitchedSystem> mySwitchedSys(new SwitchedSystem(switchedSystems, cm_seq, controller));
    Integrator<State_t> integrator(mySwitchedSys, EULER);

    // Forward Integrate
    State_t x, x_next;
    x.setZero();
    x[0] = 1.0;
    double dt = 0.10;
    double t = cm_seq.getStartTimeFromIdx(0);
    if (VERBOSE)
    {
        std::cout << "x(" << t << "): " << x.transpose() << std::endl;
    }
    while (t < cm_seq.getTotalDuration())
    {
        integrator.integrate_n_steps(x, t, 1, dt);
        t += dt;

        if (VERBOSE)
        {
            std::cout << "x(" << t << "): " << x.transpose() << std::endl;
        }

        if (t < 1.5)
        {
            ASSERT_EQ(x[1], 0.0);  // System 1 starts in equilibrium for x[1]
        }
        else if (t > (1.5 + dt))
        {
            ASSERT_TRUE(x[1] > 0.0);  // System 2 removes it from the equilibrium
        }
    }

    // Linearization
    sysPtr1->setController(controller);
    sysPtr2->setController(controller);
    LinearizerSystemPtr linSys1(new SystemLinearizer(sysPtr1));
    LinearizerSystemPtr linSys2(new SystemLinearizer(sysPtr2));

    // Switched Linearization
    SwitchedLinearSystems switchedLinearSystems;
    switchedLinearSystems.push_back(linSys1);
    switchedLinearSystems.push_back(linSys2);
    SwitchedLinearSystem switchedLinearSystem(switchedLinearSystems, cm_seq);

    // Test linearizations
    SwitchedLinearSystem::state_matrix_t A;
    SwitchedLinearSystem::state_control_matrix_t B;
    x.setZero();
    x[0] = 1.0;
    t = cm_seq.getStartTimeFromIdx(0);
    while (t < cm_seq.getTotalDuration())
    {
        A = switchedLinearSystem.getDerivativeState(x, u, t);
        B = switchedLinearSystem.getDerivativeControl(x, u, t);

        if (VERBOSE)
        {
            std::cout << "A(" << t << "):\n" << A << std::endl;
        }

        if (t < 1.5 || t > 3.5)
        {
            ASSERT_EQ(A(1, 1), 0.0);  // System 1 linearization
        }
        else if (t > 1.5 && t < 3.5)
        {
            ASSERT_EQ(A(1, 1), -5.0);  // System 2 linearization
        }

        t += dt;
    }
}


TEST(SwitchedControlledSystemTest, SwitchedControlledSystem_discrete_time)
{
    // Convenience aliases
    using System = TestNonlinearSystem<DISCRETE_TIME>;
    using State_t = EuclideanState<System::STATE_DIM>;
    using SwitchedSystem = SwitchedControlledSystem<State_t, System::CONTROL_DIM, DISCRETE_TIME>;
    using SystemPtr = SwitchedSystem::SystemPtr;
    using SwitchedSystems = SwitchedSystem::SwitchedSystems;
    using ConstantController = ConstantController<State_t, System::CONTROL_DIM, DISCRETE_TIME>;
    using Controller = std::shared_ptr<Controller<State_t, System::CONTROL_DIM, DISCRETE_TIME>>;

    // Setup systems
    SystemPtr sysPtr1(new System(0.0));
    SystemPtr sysPtr2(new System(1.0));
    SwitchedSystems switchedSystems;
    switchedSystems.push_back(sysPtr1);
    switchedSystems.push_back(sysPtr2);

    // Setup mode sequence
    DiscreteModeSequence dm_seq;
    dm_seq.addPhase(0, 2);  // phase 0, t in [0, 2)
    dm_seq.addPhase(1, 3);  // phase 1, t in [2, 5)
    dm_seq.addPhase(0, 1);  // phase 2, t in [5, 6)

    // Setup Constant Controller
    System::control_vector_t u;
    u[0] = 1.0;
    Controller controller(new ConstantController(u));

    // Construct Switched System
    std::shared_ptr<SwitchedSystem> mySwitchedSys(new SwitchedSystem(switchedSystems, dm_seq, controller));

    // Forward Integrate
    System::state_vector_t x, x_incr;
    x.setZero();
    x[0] = 1.0;
    for (int t = dm_seq.getStartTimeFromIdx(0); t < dm_seq.getTotalDuration(); t++)
    {
        mySwitchedSys->computeDynamics(x, t, x_incr);
        x += x_incr;
    }
}


/*!
 *  SwitchingControlledSystemTest.cpp
 *
 *  Test basic functionality of switching continuous controlled systems
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
