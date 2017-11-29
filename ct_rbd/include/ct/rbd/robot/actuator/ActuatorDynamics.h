/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/JointState.h>

namespace ct {
namespace rbd {


/*!
 * This class covers the actuator dynamics for a robot, i.e. not the dynamics of a single actuator, but
 * the dynamics of the collection of all actuators in the system
 * The actuators are assumed to form a symplectic system.
 */
template <size_t NJOINTS, size_t ACT_STATE_DIM = 2 * NJOINTS, typename SCALAR = double>
class ActuatorDynamics : public core::SymplecticSystem<ACT_STATE_DIM / 2, ACT_STATE_DIM / 2, NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t ACT_POS_DIM = ACT_STATE_DIM / 2;
    static const size_t ACT_VEL_DIM = ACT_STATE_DIM / 2;

    typedef ct::core::StateVector<ACT_STATE_DIM, SCALAR> act_state_vector_t;
    typedef ct::core::StateVector<ACT_POS_DIM, SCALAR> act_pos_vector_t;
    typedef ct::core::StateVector<ACT_VEL_DIM, SCALAR> act_vel_vector_t;


    ActuatorDynamics(){};

    virtual ~ActuatorDynamics(){};

    virtual ActuatorDynamics<NJOINTS, ACT_STATE_DIM, SCALAR>* clone() const override = 0;

    virtual void computePdot(const act_state_vector_t& x,
        const act_vel_vector_t& v,
        const core::ControlVector<NJOINTS, SCALAR>& control,
        act_pos_vector_t& pDot) override = 0;


    virtual void computeVdot(const act_state_vector_t& x,
        const act_pos_vector_t& p,
        const core::ControlVector<NJOINTS, SCALAR>& control,
        act_vel_vector_t& vDot) override = 0;


    //! output equation of the actuator
    /**
	 * Algebraic output equation for the actuator system, translating the current actuator state and the current robot state into
	 * an output control vector.
	 * Example: in an RBD setting, the controlOutput may be the actual joint torques.
	 * @param robotState current RBD state of the robot
	 * @param actState current state of the actuators
	 * @param controlOutput control output (output side of actuator)
	 */
    virtual core::ControlVector<NJOINTS, SCALAR> computeControlOutput(
        const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
        const act_state_vector_t& actState) = 0;
};


}  // namespace rbd
}  // namespace ct
