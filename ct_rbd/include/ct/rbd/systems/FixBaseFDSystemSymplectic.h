/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FixBaseSystemBase.h"
#include <ct/rbd/robot/actuator/ActuatorDynamicsSymplectic.h>


#define ACTUATOR_DYNAMICS_ENABLED    \
    template <size_t ACT_STATE_DIMS> \
    typename std::enable_if<(ACT_STATE_DIMS > 0), void>::type
#define ACTUATOR_DYNAMICS_DISABLED   \
    template <size_t ACT_STATE_DIMS> \
    typename std::enable_if<(ACT_STATE_DIMS <= 0), void>::type

namespace ct {
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics.
 *
 * A fix base rigid body system that uses forward dynamics. The control input vector is assumed to consist of
 *  - joint torques and
 *  - end effector forces expressed in the world frame
 *
 * The state vectors of the pure RBD system and the actuator dynamics are split such that
 * the overall system can be considered symplectic. The state vector
 * therefore reads as
 *
 * \f[
 * ( \theta_{joints}^\top p_{act}^\top \dot \theta{joints}^\top v_{act}^\top )^\top
 * \f]
 *
 * \warning Make sure the actuator dynamics can be written as a symplectic system
 *  in case of using a symplectic integrator.
 */
template <class RBDDynamics, size_t ACT_STATE_DIM = 0, bool EE_ARE_CONTROL_INPUTS = false>
class FixBaseFDSystemSymplectic
    : public RBDSystem<RBDDynamics, false>,
      public core::SymplecticSystem<RBDDynamics::NJOINTS + ACT_STATE_DIM / 2,    // p-dim of combined system
          RBDDynamics::NJOINTS + ACT_STATE_DIM / 2,                              // v-dim of combined system
          RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3,  // input-dim of combined system
          typename RBDDynamics::SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t N_EE = RBDDynamics::N_EE;                         // number of end-effectors
    static const size_t STATE_DIM = RBDDynamics::NSTATE + ACT_STATE_DIM;  // RBD state-dim plus actuator state-dim
    static const size_t CONTROL_DIM =
        RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3;  // torques plus EE-forces
    static const size_t ACTUATOR_STATE_DIM = ACT_STATE_DIM;
    static const size_t NJOINTS = RBDDynamics::NJOINTS;

    using Dynamics = RBDDynamics;
    using SCALAR = typename RBDDynamics::SCALAR;
    using Base = core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>;
    using ActuatorDynamics_t = ActuatorDynamicsSymplectic<RBDDynamics::NJOINTS, ACT_STATE_DIM, SCALAR>;

    // typedefs state and controls on all occurring dimensions
    using state_vector_full_t = core::StateVector<STATE_DIM, SCALAR>;
    using control_vector_full_t = core::ControlVector<CONTROL_DIM, SCALAR>;
    using state_vector_rbd_t = core::StateVector<RBDDynamics::NSTATE, SCALAR>;
    using state_vector_act_t = core::StateVector<ACTUATOR_STATE_DIM, SCALAR>;
    using p_vector_full_t = core::StateVector<STATE_DIM / 2, SCALAR>;
    using v_vector_full_t = core::StateVector<STATE_DIM / 2, SCALAR>;
    using p_vector_rbd_t = core::StateVector<RBDDynamics::NSTATE / 2, SCALAR>;
    using v_vector_rbd_t = core::StateVector<RBDDynamics::NSTATE / 2, SCALAR>;
    using p_vector_act_t = core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR>;
    using v_vector_act_t = core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR>;


    //! constructor
    FixBaseFDSystemSymplectic()
        : Base(core::SYSTEM_TYPE::SECOND_ORDER), dynamics_(RBDDynamics()), actuatorDynamics_(nullptr)
    {
        basePose_.setIdentity();
    }

    //! constructor including actuator dynamics
    FixBaseFDSystemSymplectic(std::shared_ptr<ActuatorDynamics_t> actuatorDynamics)
        : Base(core::SYSTEM_TYPE::SECOND_ORDER), dynamics_(RBDDynamics()), actuatorDynamics_(actuatorDynamics)
    {
        basePose_.setIdentity();
    }

    /*!
     * @brief copy constructor
	 *
	 * @param arg instance of FixBaseFDSystemSymplectic to be copied.
	 *
	 * \note takes care of explicitly cloning actuatorDynamics, if existent
	 */
    FixBaseFDSystemSymplectic(const FixBaseFDSystemSymplectic& arg)
        : Base(arg), basePose_(arg.basePose_), dynamics_(RBDDynamics())
    {
        if (arg.actuatorDynamics_)
        {
            actuatorDynamics_ = std::shared_ptr<ActuatorDynamics_t>(arg.actuatorDynamics_->clone());
        }
    }

    //! destructor
    virtual ~FixBaseFDSystemSymplectic() {}
    //! get dynamics
    virtual RBDDynamics& dynamics() override { return dynamics_; }
    //! get dynamics (const)
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    //! compute position derivatives, for both RBD system and actuator dynamics, if applicable
    virtual void computePdot(const state_vector_full_t& x,
        const v_vector_full_t& v,
        const control_vector_full_t& controlIn,
        p_vector_full_t& pDot) override
    {
        // the top rows hold the RBD velocities ...
        pDot.template topRows<RBDDynamics::NJOINTS>() = v.template topRows<RBDDynamics::NJOINTS>();

        // full joint state vector
        typename RBDDynamics::JointState_t jState = jointStateFromVector(x);

        // compute he pDot for the actuator part
        computeActuatorPdot<ACT_STATE_DIM>(jState, x, v, controlIn, pDot);
    }


    ACTUATOR_DYNAMICS_ENABLED computeActuatorPdot(const typename RBDDynamics::JointState_t& jState,
        const state_vector_full_t& x,
        const v_vector_full_t& v,
        const control_vector_full_t& controlIn,
        p_vector_full_t& pDot)
    {
        // get const references to the current actuator position, velocity, and actuator input
        const Eigen::Ref<const typename p_vector_act_t::Base> actPos =
            x.template segment<ACT_STATE_DIM / 2>(RBDDynamics::NJOINTS);
        const Eigen::Ref<const typename v_vector_act_t::Base> actVel = v.template bottomRows<ACT_STATE_DIM / 2>();
        const Eigen::Ref<const typename control_vector_full_t::Base> actControlIn =
            controlIn.template topRows<RBDDynamics::NJOINTS>();

        // assemble temporary actuator state
        state_vector_act_t actState;
        actState << actPos, actVel;

        p_vector_act_t actPdot;
        actuatorDynamics_->computePdot(jState, actState, actVel, actControlIn, actPdot);

        // ... the bottom rows hold the actuator dynamics
        pDot.template bottomRows<ACT_STATE_DIM / 2>() = actPdot;
    }


    ACTUATOR_DYNAMICS_DISABLED computeActuatorPdot(const typename RBDDynamics::JointState_t& jState,
        const state_vector_full_t& x,
        const v_vector_full_t& v,
        const control_vector_full_t& controlIn,
        p_vector_full_t& pDot)
    {
    }


    //! compute velocity derivatives, for both RBD system and actuator dynamics
    virtual void computeVdot(const state_vector_full_t& x,
        const p_vector_full_t& p,
        const control_vector_full_t& controlIn,
        v_vector_full_t& vDot) override
    {
        // temporary variable for the control (will get modified by the actuator dynamics, if applicable)
        control_vector_full_t control = controlIn;

        // extract the current RBD joint state from the state vector // todo make method to get joint state
        typename RBDDynamics::JointState_t jState = jointStateFromVector(x);

        computeActuatorVdot<ACT_STATE_DIM>(jState, x, p, controlIn, vDot, control);

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
                        control.template segment<3>(RBDDynamics::NJOINTS + i * 3), basePose_, jState.getPositions(), i);
            }
        }

        typename RBDDynamics::JointAcceleration_t jAcc;

        dynamics_.FixBaseForwardDynamics(jState, control.template head<RBDDynamics::NJOINTS>(), linkForces, jAcc);

        vDot.template topRows<RBDDynamics::NJOINTS>() = jAcc.getAcceleration();
    }


    ACTUATOR_DYNAMICS_ENABLED computeActuatorVdot(const typename RBDDynamics::JointState_t& jState,
        const state_vector_full_t& x,
        const p_vector_full_t& p,
        const control_vector_full_t& controlIn,
        v_vector_full_t& vDot,
        control_vector_full_t& controlOut)
    {
        // get references to the current actuator position, velocity and input
        const Eigen::Ref<const typename p_vector_act_t::Base> actPos =
            p.template segment<ACT_STATE_DIM / 2>(RBDDynamics::NJOINTS);
        const Eigen::Ref<const typename v_vector_act_t::Base> actVel = x.template bottomRows<ACT_STATE_DIM / 2>();
        const Eigen::Ref<const typename control_vector_full_t::Base> actControlIn =
            controlIn.template topRows<RBDDynamics::NJOINTS>();

        // assemble temporary actuator state
        state_vector_act_t actState;
        actState << actPos, actVel;

        // ... the bottom rows hold the actuator dynamics
        v_vector_act_t actVdot;
        actuatorDynamics_->computeVdot(jState, actState, actPos, actControlIn, actVdot);
        vDot.template bottomRows<ACT_STATE_DIM / 2>() = actVdot;

        // overwrite control with actuator control output as a function of current robot and actuator states
        controlOut = actuatorDynamics_->computeControlOutput(jState, actState);
    }


    ACTUATOR_DYNAMICS_DISABLED computeActuatorVdot(const typename RBDDynamics::JointState_t& jState,
        const state_vector_full_t& x,
        const p_vector_full_t& p,
        const control_vector_full_t& controlIn,
        v_vector_full_t& vDot,
        control_vector_full_t& controlOut)
    {
    }


    //! deep cloning
    virtual FixBaseFDSystemSymplectic<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>* clone() const override
    {
        return new FixBaseFDSystemSymplectic<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>(*this);
    }

    //! full vector to ct JointState
    static const typename RBDDynamics::JointState_t jointStateFromVector(const state_vector_full_t& x_full)
    {
        typename RBDDynamics::JointState_t jointState;
        jointState.getPositions() = x_full.template head<RBDDynamics::NJOINTS>();
        jointState.getVelocities() = x_full.template segment<RBDDynamics::NJOINTS>(STATE_DIM / 2);
        return jointState;
    }

    //! transform control systems state vector to a RBDState
    static const typename RBDDynamics::RBDState_t RBDStateFromVector(const state_vector_full_t& x_full,
        const tpl::RigidBodyPose<SCALAR>& basePose = tpl::RigidBodyPose<SCALAR>())
    {
        typename RBDDynamics::RBDState_t rbdState;
        rbdState.setZero();
        rbdState.basePose() = basePose;
        rbdState.joints() = jointStateFromVector(x_full);
        return rbdState;
    }

    //! transform control systems state vector to a RBDState
    static const state_vector_rbd_t rbdStateFromVector(const state_vector_full_t& x_full)
    {
        state_vector_rbd_t x_small;
        x_small.template head<NJOINTS>() = x_full.template head<NJOINTS>();
        x_small.template tail<NJOINTS>() = x_full.template segment<NJOINTS>(STATE_DIM / 2);
        return x_small;
    }

    //! transform control systems state vector to the pure actuator state
    static const state_vector_act_t actuatorStateFromVector(const state_vector_full_t& state)
    {
        state_vector_act_t actState;
        actState.template topRows<ACT_STATE_DIM / 2>() =
            state.template segment<ACT_STATE_DIM / 2>(RBDDynamics::NSTATE / 2);
        actState.template bottomRows<ACT_STATE_DIM / 2>() = state.template bottomRows<ACT_STATE_DIM / 2>();
        return actState;
    }

    //! transform from "reduced" to full coordinates including actuator dynamics
    static const state_vector_full_t toFullState(const state_vector_rbd_t& x_rbd,
        const state_vector_act_t& x_act = state_vector_act_t::Zero())
    {
        state_vector_full_t fullState;
        fullState.template head<NJOINTS>() = x_rbd.template head<NJOINTS>();
        fullState.template segment<ACTUATOR_STATE_DIM / 2>(NJOINTS) = x_act.template head<ACTUATOR_STATE_DIM / 2>();
        fullState.template segment<NJOINTS>(STATE_DIM / 2) = x_rbd.template tail<NJOINTS>();
        fullState.template tail<ACTUATOR_STATE_DIM / 2>() = x_act.template tail<ACTUATOR_STATE_DIM / 2>();
        return fullState;
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

#undef ACTUATOR_DYNAMICS_ENABLED
#undef ACTUATOR_DYNAMICS_DISABLED
