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
 *
 * \note This class does on purpose not derive from ControlledSystem, as it requires the full robot state
 *
 * @tparam state dimensions of all actuators in the system together
 * @tparam number of joints in the robot
 * @tparam primitive scalar type, eg. double
 */
template <size_t ACT_STATE_DIMS, size_t NJOINTS, typename SCALAR = double>
class ActuatorDynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t ACT_STATE_DIM = ACT_STATE_DIMS;
    static const size_t ACT_CONTROL_DIM = NJOINTS;

    typedef ct::core::StateVector<ACT_STATE_DIMS, SCALAR> act_state_vector_t;
    typedef ct::core::ControlVector<NJOINTS, SCALAR> act_control_vector_t;

    ActuatorDynamics(){};

    virtual ~ActuatorDynamics(){};

    virtual ActuatorDynamics<ACT_STATE_DIMS, NJOINTS, SCALAR>* clone() const = 0;

    virtual void computeActuatorDynamics(const JointState<NJOINTS, SCALAR>& robotJointState,
        const ct::core::StateVector<ACT_STATE_DIMS, SCALAR>& actuatorState,
        const SCALAR& t,
        const ct::core::ControlVector<NJOINTS, SCALAR>& control,
        ct::core::StateVector<ACT_STATE_DIMS, SCALAR>& derivative) = 0;

    /**
     * @brief output equation of the actuator
     *
	 * Algebraic output equation for the actuator system, translating the current actuator state and the current robot state into
	 * an output control vector.
	 * Example: in an RBD setting, the controlOutput may be the actual joint torques.
	 *
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
