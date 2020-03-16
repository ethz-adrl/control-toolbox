/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;
using std::shared_ptr;


TEST(SwitchingTest, ContinuousModeSequence)
{
    ContinuousModeSequence cm_seq(-0.5);
    cm_seq.addPhase(0, 0.25);  // phase 0, t in [-0.50, -0.25)
    cm_seq.addPhase(4, 0.50);  // phase 1, t in [-0.25,  0.25)
    cm_seq.addPhase(2, 1.00);  // phase 2, t in [ 0.25,  1.25)
    cm_seq.addPhase(1, 0.75);  // phase 3, t in [ 1.25,  2.00)
    ASSERT_EQ(cm_seq.getNumPhases(), 4);
    ASSERT_EQ(cm_seq.getNumSwitches(), 3);
    ASSERT_DOUBLE_EQ(cm_seq.getTotalDuration(), 2.50);
    ASSERT_DOUBLE_EQ(cm_seq.getStartTimeFromIdx(1), -0.25);
    ASSERT_DOUBLE_EQ(cm_seq.getEndTimeFromIdx(2), 1.25);

    // Test Phases
    ASSERT_EQ(cm_seq.getPhaseFromIdx(0), 0);
    ASSERT_EQ(cm_seq.getPhaseFromIdx(1), 4);
    ASSERT_EQ(cm_seq.getPhaseFromIdx(2), 2);
    ASSERT_EQ(cm_seq.getPhaseFromIdx(3), 1);

    // Test Switch Event
    auto cm_switch = cm_seq.getSwitchEventFromIdx(0);
    ASSERT_EQ(cm_switch.pre_phase, 0);
    ASSERT_EQ(cm_switch.post_phase, 4);
    ASSERT_DOUBLE_EQ(cm_switch.switch_time, -0.25);

    // Test Time indexing
    ASSERT_EQ(cm_seq.getIdxFromTime(-1.0), 0);  // Before first point
    ASSERT_EQ(cm_seq.getIdxFromTime(-0.5), 0);  // Exactly on first point
    ASSERT_EQ(cm_seq.getIdxFromTime(0.24), 1);  // Between two points
    ASSERT_EQ(cm_seq.getIdxFromTime(0.25), 2);  // Exactly on a boundary
    ASSERT_EQ(cm_seq.getIdxFromTime(2.00), 3);  // Exactly on end point
    ASSERT_EQ(cm_seq.getIdxFromTime(2.01), 3);  // After end point
}

TEST(SwitchingTest, DiscreteModeSequence)
{
    DiscreteModeSequence dm_seq;
    dm_seq.addPhase(0, 2);  // phase 0, t in [0, 2)
    dm_seq.addPhase(4, 3);  // phase 1, t in [2, 5)
    dm_seq.addPhase(2, 1);  // phase 2, t in [5, 6)
    dm_seq.addPhase(1, 2);  // phase 3, t in [6, 8)
    ASSERT_EQ(dm_seq.getNumPhases(), 4);
    ASSERT_EQ(dm_seq.getNumSwitches(), 3);
    ASSERT_EQ(dm_seq.getTotalDuration(), 8);
    ASSERT_EQ(dm_seq.getStartTimeFromIdx(1), 2);
    ASSERT_EQ(dm_seq.getEndTimeFromIdx(2), 6);

    // Test Phases
    ASSERT_EQ(dm_seq.getPhaseFromIdx(0), 0);
    ASSERT_EQ(dm_seq.getPhaseFromIdx(1), 4);
    ASSERT_EQ(dm_seq.getPhaseFromIdx(2), 2);
    ASSERT_EQ(dm_seq.getPhaseFromIdx(3), 1);

    // Test Switch Event
    auto dm_switch = dm_seq.getSwitchEventFromIdx(0);
    ASSERT_EQ(dm_switch.pre_phase, 0);
    ASSERT_EQ(dm_switch.post_phase, 4);
    ASSERT_EQ(dm_switch.switch_time, 2);

    // Test Time indexing
    ASSERT_EQ(dm_seq.getIdxFromTime(-1.0), 0);  // Before first point
    ASSERT_EQ(dm_seq.getIdxFromTime(0), 0);     // Exactly on first point
    ASSERT_EQ(dm_seq.getIdxFromTime(4), 1);     // Between two points
    ASSERT_EQ(dm_seq.getIdxFromTime(5), 2);     // Exactly on a boundary
    ASSERT_EQ(dm_seq.getIdxFromTime(8), 3);     // Exactly on end point
    ASSERT_EQ(dm_seq.getIdxFromTime(9), 3);     // After end point
}

TEST(SwitchingTest, SwitchedSystem)
{
    // Create two separate systems
    double w_n1 = 0.1;
    double w_n2 = 0.2;
    double zeta1 = 0.0;
    double zeta2 = 0.0;
    shared_ptr<SecondOrderSystem> oscillator1 = shared_ptr<SecondOrderSystem>(new SecondOrderSystem(w_n1, zeta1));
    shared_ptr<SecondOrderSystem> oscillator2 = shared_ptr<SecondOrderSystem>(new SecondOrderSystem(w_n2, zeta2));
    oscillator1->checkParameters();
    oscillator2->checkParameters();

    // Combine to a two mode switched system
    using SwitchedSecondOrderSystem = Switched<shared_ptr<SecondOrderSystem>>;
    SwitchedSecondOrderSystem ssos;
    ssos.push_back(oscillator1);
    ssos.push_back(oscillator2);

    // Define Mode sequence
    ContinuousModeSequence cm_seq;
    cm_seq.addPhase(1, 0.25);  // phase 0, t in [0.00, 0.25)
    cm_seq.addPhase(0, 0.50);  // phase 1, t in [0.25, 0.75)
    cm_seq.addPhase(1, 0.25);  // phase 2, t in [0.75, 1.00)

    // Test the switched system
    StateVector<2> state, derivative;
    state(0) = 1.0;
    state(1) = 1.0;
    ControlVector<1> control;
    control.setZero();

    for (double t = 0.0; t < cm_seq.getTotalDuration(); t += 0.1)
    {
        auto phase = cm_seq.getPhaseFromTime(t);
        ssos[phase]->computeControlledDynamics(state, t, control, derivative);
        if (0.25 <= t and t < 0.75)
        {
            // Check if system 1 is active based on analytic solution
            ASSERT_DOUBLE_EQ(derivative(1), control(0) - 2.0 * zeta1 * w_n1 * state(1) - w_n1 * w_n1 * state(0));
        }
        else if (t < 0.25 or t > 0.75)
        {
            // Check if system 2 is active based on analytic solution
            ASSERT_DOUBLE_EQ(derivative(1), control(0) - 2.0 * zeta2 * w_n2 * state(1) - w_n2 * w_n2 * state(0));
        }
    }
}

/*!
 *  SwitchingTest.cpp
 *
 *  Test basic functionality of switching logic
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}