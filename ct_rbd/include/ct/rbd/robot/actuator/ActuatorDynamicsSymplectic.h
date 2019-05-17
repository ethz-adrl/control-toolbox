/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
template <size_t NJOINTS, size_t ACT_STATE_DIMS = 2 * NJOINTS, typename SCALAR = double>
class ActuatorDynamicsSymplectic
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t ACT_STATE_DIM = ACT_STATE_DIMS;
    static const size_t ACT_POS_DIM = ACT_STATE_DIMS / 2;
    static const size_t ACT_VEL_DIM = ACT_STATE_DIMS / 2;

    typedef ct::core::StateVector<ACT_STATE_DIMS, SCALAR> act_state_vector_t;
    typedef ct::core::StateVector<ACT_POS_DIM, SCALAR> act_pos_vector_t;
    typedef ct::core::StateVector<ACT_VEL_DIM, SCALAR> act_vel_vector_t;


    ActuatorDynamicsSymplectic(){};

    virtual ~ActuatorDynamicsSymplectic(){};

    virtual ActuatorDynamicsSymplectic<NJOINTS, ACT_STATE_DIMS, SCALAR>* clone() const override = 0;

    virtual void computePdot(const JointState<NJOINTS, SCALAR>& robotJointState,
        const act_state_vector_t& x,
        const act_vel_vector_t& v,
        const core::ControlVector<NJOINTS, SCALAR>& control,
        act_pos_vector_t& pDot) = 0;


    virtual void computeVdot(const JointState<NJOINTS, SCALAR>& robotJointState,
        const act_state_vector_t& x,
        const act_pos_vector_t& p,
        const core::ControlVector<NJOINTS, SCALAR>& control,
        act_vel_vector_t& vDot) = 0;


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
        const JointState<NJOINTS, SCALAR>& robotJointState,
        const act_state_vector_t& actState) = 0;

    /*!
     * @brief reconstruct actuator state from a desired control output and robot joint state (e.g. for initialization)
     * @return the actuator state resulting in the desired control output
     */
    virtual ct::core::StateVector<ACT_STATE_DIMS, SCALAR> computeStateFromOutput(
        const JointState<NJOINTS, SCALAR>& refRobotJointState,
        const core::ControlVector<NJOINTS, SCALAR>& refControl) = 0;
};


}  // namespace rbd
}  // namespace ct
