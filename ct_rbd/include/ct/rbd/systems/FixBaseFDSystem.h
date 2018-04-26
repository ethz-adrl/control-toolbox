/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/robot/actuator/ActuatorDynamics.h>
#include <ct/rbd/state/FixBaseRobotState.h>

#include "RBDSystem.h"

namespace ct {
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics.
 *
 * A fix base rigid body system that uses forward dynamics. The control input vector is assumed to consist of
 *  - joint torques and
 *  - end effector forces expressed in the world frame
 *
 * The overall state vector is arranged in the order
 * - joint positions
 * - joint velocities
 * - actuator state
 *
 * \warning when modelled with RobCoGen, the base pose must be rotated "against gravity" (RobCoGen modelling assumption)
 */
template <class RBDDynamics, size_t ACT_STATE_DIM = 0, bool EE_ARE_CONTROL_INPUTS = false>
class FixBaseFDSystem
    : public RBDSystem<RBDDynamics, false>,
      public core::ControlledSystem<RBDDynamics::NSTATE + ACT_STATE_DIM,         // state-dim
          RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3,  // input-dim of combined system
          typename RBDDynamics::SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! number of end-effectors
    static const size_t N_EE = RBDDynamics::N_EE;
    //! rigid body system state-dim plus actuator state-dim
    static const size_t STATE_DIM = RBDDynamics::NSTATE + ACT_STATE_DIM;
    //! joint torques plus end-effector forces
    static const size_t NJOINTS = RBDDynamics::NJOINTS;
    //! we assume one control input per joint
    static const size_t CONTROL_DIM = NJOINTS + EE_ARE_CONTROL_INPUTS * N_EE * 3;
    //! convenience definition
    static const size_t ACTUATOR_STATE_DIM = ACT_STATE_DIM;

    using Dynamics = RBDDynamics;
    using SCALAR = typename RBDDynamics::SCALAR;
    using Base = core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>;
    using ActuatorDynamics_t = ActuatorDynamics<ACT_STATE_DIM, NJOINTS, SCALAR>;
    using RigidBodyPose_t = tpl::RigidBodyPose<SCALAR>;
    using FixBaseRobotState_t = FixBaseRobotState<NJOINTS, ACT_STATE_DIM, SCALAR>;

    // typedefs state and controls
    using state_vector_t = typename FixBaseRobotState_t::state_vector_t;
    using actuator_state_vector_t = typename FixBaseRobotState_t::actuator_state_vector_t;
    using control_vector_t = core::ControlVector<CONTROL_DIM, SCALAR>;
    using JointAcceleration_t = JointAcceleration<NJOINTS, SCALAR>;


    //! constructor
    /*!
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
    FixBaseFDSystem(const RigidBodyPose_t& basePose = RigidBodyPose_t())
        : Base(), basePose_(basePose), dynamics_(RBDDynamics()), actuatorDynamics_(nullptr)
    {
    }
    //! constructor including actuator dynamics
    /*!
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
    FixBaseFDSystem(std::shared_ptr<ActuatorDynamics_t> actuatorDynamics,
        const RigidBodyPose_t& basePose = RigidBodyPose_t())
        : Base(), basePose_(basePose), dynamics_(RBDDynamics()), actuatorDynamics_(actuatorDynamics)
    {
        basePose_.setIdentity();
    }

    /*!
     * @brief copy constructor
	 *
	 * @param arg instance of FixBaseFDSystem to be copied.
	 *
	 * \note takes care of explicitly cloning actuatorDynamics, if existent
	 */
    FixBaseFDSystem(const FixBaseFDSystem& arg) : Base(arg), basePose_(arg.basePose_), dynamics_(RBDDynamics())
    {
        if (arg.actuatorDynamics_)
        {
            actuatorDynamics_ = std::shared_ptr<ActuatorDynamics_t>(arg.actuatorDynamics_->clone());
        }
    }

    //! destructor
    virtual ~FixBaseFDSystem() {}
    //! get dynamics
    virtual RBDDynamics& dynamics() override { return dynamics_; }
    //! get dynamics (const)
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    //! compute the controlled dynamics of the fixed base robotic system
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        ct::core::StateVector<STATE_DIM, SCALAR>& derivative)
    {
        FixBaseRobotState_t robotState(state);

        // map the joint velocities (unchanged, no damping)
        derivative.template topRows<NJOINTS>() = robotState.joints().getVelocities();

        // temporary variable for the control (will get modified by the actuator dynamics, if applicable)
        control_vector_t control = controlIn;

        // compute actuator dynamics and their control output
        computeActuatorDynamics(robotState, t, controlIn, derivative, control);

        // Cache updated rbd state
        typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

        // add end effector forces as control inputs (if applicable)
        if (EE_ARE_CONTROL_INPUTS == true)
        {
            for (size_t i = 0; i < RBDDynamics::N_EE; i++)
            {
                auto endEffector = dynamics_.kinematics().getEndEffector(i);
                size_t linkId = endEffector.getLinkId();
                linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
                    dynamics_.kinematics().mapForceFromWorldToLink3d(
                        control.template segment<3>(RBDDynamics::NJOINTS + i * 3), basePose_,
                        robotState.joints().getPositions(), i);
            }
        }

        typename RBDDynamics::JointAcceleration_t jAcc;

        dynamics_.FixBaseForwardDynamics(
            robotState.joints(), control.template head<RBDDynamics::NJOINTS>(), linkForces, jAcc);

        derivative.template segment<RBDDynamics::NJOINTS>(NJOINTS) = jAcc.getAcceleration();
    }

    //! evaluate the actuator dynamics if actuators are enabled
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeActuatorDynamics(const FixBaseRobotState_t& robotState,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& stateDerivative,
        control_vector_t& controlOut)
    {
        // get references to the current actuator position, velocity and input
        const Eigen::Ref<const typename control_vector_t::Base> actControlIn = controlIn.template topRows<NJOINTS>();

        actuator_state_vector_t actStateDerivative;  // todo use vector block for this?
        actuatorDynamics_->computeActuatorDynamics(
            robotState.joints(), robotState.actuatorState(), t, actControlIn, actStateDerivative);

        stateDerivative.template tail<ACTUATOR_STATE_DIM>() = actStateDerivative;

        // overwrite control with actuator control output as a function of current robot and actuator states
        controlOut = actuatorDynamics_->computeControlOutput(robotState.joints(), robotState.actuatorState());
    }

    //! do nothing if actuators disabled
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type computeActuatorDynamics(
        const FixBaseRobotState_t& robotState,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& stateDerivative,
        control_vector_t& controlOut)
    {
    }

    //! deep cloning
    virtual FixBaseFDSystem<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>* clone() const override
    {
        return new FixBaseFDSystem<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>(*this);
    }

    //! get pointer to actuator dynamics
    std::shared_ptr<ActuatorDynamics_t> getActuatorDynamics() { return actuatorDynamics_; }
    //! compute inverse dynamics torques
    ct::core::ControlVector<NJOINTS> computeIDTorques(const JointState<NJOINTS, SCALAR>& jState,
        const JointAcceleration_t& jAcc = JointAcceleration_t(Eigen::Matrix<SCALAR, NJOINTS, 1>::Zero()))
    {
        ct::core::ControlVector<NJOINTS> u;
        dynamics_.FixBaseID(jState, jAcc, u);
        return u;
    }

    //! if actuator dynamics enabled, this method allows to design a consistent actuator state
    template <typename T = typename FixBaseRobotState_t::actuator_state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeConsistentActuatorState(
        const JointState<NJOINTS, SCALAR>& jStateRef,
        const ct::core::ControlVector<NJOINTS>& torqueRef)
    {
        return actuatorDynamics_->computeStateFromOutput(jStateRef, torqueRef);
    }

    //! if actuator dynamics enabled, this method allows to design a consistent actuator state
    template <typename T = typename FixBaseRobotState_t::actuator_state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM>0), T>::type computeConsistentActuatorState(
        const JointState<NJOINTS, SCALAR>& jStateRef)
    {
        const ct::core::ControlVector<NJOINTS> torqueRef = computeIDTorques(jStateRef);
        return computeConsistentActuatorState(jStateRef, torqueRef);
    }


private:
    //! a "dummy" base pose which sets the robot's "fixed" position in the world
    tpl::RigidBodyPose<SCALAR> basePose_;

    //! rigid body dynamics container
    RBDDynamics dynamics_;

    //! pointer to the actuator dynamics
    std::shared_ptr<ActuatorDynamics_t> actuatorDynamics_;
};

}  // namespace rbd
}  // namespace ct
