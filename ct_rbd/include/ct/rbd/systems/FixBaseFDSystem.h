/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/robot/actuator/ActuatorDynamics.h>

#include "RBDSystem.h"


#define ACTUATOR_DYNAMICS_ENABLED   \
    template <size_t ACT_STATE_DIM> \
    typename std::enable_if<(ACT_STATE_DIM > 0), void>::type
#define ACTUATOR_DYNAMICS_DISABLED  \
    template <size_t ACT_STATE_DIM> \
    typename std::enable_if<(ACT_STATE_DIM <= 0), void>::type

namespace ct {
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end effector forces expressed in the world.
 *
 * We split the state vectors of the pure rbd system and the actuator dynamics such that
 * the overall system can be straight-forwardly considered symplectic.
 */
template <class RBDDynamics, size_t ACTUATOR_STATE_DIM = 0, bool EE_ARE_CONTROL_INPUTS = false>
class FixBaseFDSystem
    : public RBDSystem<RBDDynamics, false>,
      public core::SymplecticSystem<RBDDynamics::NJOINTS +
                                        ACTUATOR_STATE_DIM / 2,  // position dimension of combined system
          RBDDynamics::NJOINTS + ACTUATOR_STATE_DIM / 2,         // velocity dimension of combined system
          RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3,  // input dimension of combined system
          typename RBDDynamics::SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Dynamics = RBDDynamics;

    typedef typename RBDDynamics::SCALAR SCALAR;

    const static size_t N_EE = RBDDynamics::N_EE;

    // the combined system consists of the RBD states and the actuator states
    static const size_t STATE_DIM = RBDDynamics::NSTATE + ACTUATOR_STATE_DIM;

    // the control inputs are the inputs to the actuators / joint torques + EE forces, if applicable
    static const size_t CONTROL_DIM = RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3;

    typedef core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR> Base;


    //! constructor
    FixBaseFDSystem() : Base(core::SYSTEM_TYPE::SECOND_ORDER), dynamics_(RBDDynamics()), actuatorDynamics_(nullptr)
    {
        basePose_.setIdentity();
    }


    //! constructor including actuator dynamics
    FixBaseFDSystem(
        std::shared_ptr<ActuatorDynamics<RBDDynamics::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>> actuatorDynamics)
        : Base(core::SYSTEM_TYPE::SECOND_ORDER), dynamics_(RBDDynamics()), actuatorDynamics_(actuatorDynamics)
    {
        basePose_.setIdentity();
    }

    //! copy constructor
    /*!
	 * take care of explicitly cloning actuatorDynamics, if existent
	 * @param arg instance of FixBaseFDSystem to be copied.
	 */
    FixBaseFDSystem(const FixBaseFDSystem& arg) : Base(arg), basePose_(arg.basePose_), dynamics_(RBDDynamics())
    {
        if (arg.actuatorDynamics_)
        {
            actuatorDynamics_ = std::shared_ptr<ActuatorDynamics<RBDDynamics::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>>(
                arg.actuatorDynamics_->clone());
        }
    }

    //! destructor
    virtual ~FixBaseFDSystem() {}
    //! get dynamics
    virtual RBDDynamics& dynamics() override { return dynamics_; }
    //! get dynamics (const)
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    //! compute position derivatives, for both RBD system and actuator dynamics, if applicable
    virtual void computePdot(const core::StateVector<STATE_DIM, SCALAR>& x,
        const core::StateVector<STATE_DIM / 2, SCALAR>& v,
        const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        core::StateVector<STATE_DIM / 2, SCALAR>& pDot) override
    {
        // the top rows hold the RBD velocities ...
        pDot.template topRows<RBDDynamics::NJOINTS>() = v.template topRows<RBDDynamics::NJOINTS>();

        computeActuatorPdot<ACTUATOR_STATE_DIM>(x, v, controlIn, pDot);
    }


    ACTUATOR_DYNAMICS_ENABLED computeActuatorPdot(const core::StateVector<STATE_DIM, SCALAR>& x,
        const core::StateVector<STATE_DIM / 2, SCALAR>& v,
        const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        core::StateVector<STATE_DIM / 2, SCALAR>& pDot)
    {
        // get const references to the current actuator position, velocity, and actuator input
        const Eigen::Ref<const typename core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR>::Base> actPos =
            x.template segment<ACTUATOR_STATE_DIM / 2>(RBDDynamics::NJOINTS);
        const Eigen::Ref<const typename core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR>::Base> actVel =
            v.template bottomRows<ACTUATOR_STATE_DIM / 2>();
        const Eigen::Ref<const typename core::ControlVector<RBDDynamics::NJOINTS>::Base> actControlIn =
            controlIn.template topRows<RBDDynamics::NJOINTS>();

        // assemble temporary actuator state
        core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actState;
        actState << actPos, actVel;

        core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR> actPdot;
        actuatorDynamics_->computePdot(actState, actVel, actControlIn, actPdot);

        // ... the bottom rows hold the actuator dynamics
        pDot.template bottomRows<ACTUATOR_STATE_DIM / 2>() = actPdot;
    }


    ACTUATOR_DYNAMICS_DISABLED computeActuatorPdot(const core::StateVector<STATE_DIM, SCALAR>& x,
        const core::StateVector<STATE_DIM / 2, SCALAR>& v,
        const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        core::StateVector<STATE_DIM / 2, SCALAR>& pDot)
    {
    }


    //! compute velocity derivatives, for both RBD system and actuator dynamics
    virtual void computeVdot(const core::StateVector<STATE_DIM, SCALAR>& x,
        const core::StateVector<STATE_DIM / 2, SCALAR>& p,
        const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        core::StateVector<STATE_DIM / 2, SCALAR>& vDot) override
    {
        // temporary variable for the control (will get modified by the actuator dynamics, if applicable)
        core::ControlVector<CONTROL_DIM, SCALAR> control = controlIn;

        // extract the current RBD joint state from the state vector
        typename RBDDynamics::JointState_t jState;
        jState.setZero();
        jState.getPositions() = p.template topRows<RBDDynamics::NJOINTS>();
        jState.getVelocities() = x.template segment<RBDDynamics::NJOINTS>(STATE_DIM / 2);

        computeActuatorVdot<ACTUATOR_STATE_DIM>(jState, x, p, controlIn, vDot, control);

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
        const core::StateVector<STATE_DIM, SCALAR>& x,
        const core::StateVector<STATE_DIM / 2, SCALAR>& p,
        const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        core::StateVector<STATE_DIM / 2, SCALAR>& vDot,
        core::ControlVector<CONTROL_DIM, SCALAR>& controlOut)
    {
        // get references to the current actuator position, velocity and input
        const Eigen::Ref<const typename core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR>::Base> actPos =
            p.template segment<ACTUATOR_STATE_DIM / 2>(RBDDynamics::NJOINTS);
        const Eigen::Ref<const typename core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR>::Base> actVel =
            x.template bottomRows<ACTUATOR_STATE_DIM / 2>();
        const Eigen::Ref<const typename core::ControlVector<RBDDynamics::NJOINTS>::Base> actControlIn =
            controlIn.template topRows<RBDDynamics::NJOINTS>();

        // assemble temporary actuator state
        core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actState;
        actState << actPos, actVel;

        // ... the bottom rows hold the actuator dynamics
        core::StateVector<ACTUATOR_STATE_DIM / 2, SCALAR> actVdot;
        actuatorDynamics_->computeVdot(actState, actPos, actControlIn, actVdot);
        vDot.template bottomRows<ACTUATOR_STATE_DIM / 2>() = actVdot;

        // overwrite control with actuator control output as a function of current robot and actuator states
        controlOut = actuatorDynamics_->computeControlOutput(jState, actState);
    }


    ACTUATOR_DYNAMICS_DISABLED computeActuatorVdot(const typename RBDDynamics::JointState_t& jState,
        const core::StateVector<STATE_DIM, SCALAR>& x,
        const core::StateVector<STATE_DIM / 2, SCALAR>& p,
        const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        core::StateVector<STATE_DIM / 2, SCALAR>& vDot,
        core::ControlVector<CONTROL_DIM, SCALAR>& controlOut)
    {
    }


    //! deep cloning
    virtual FixBaseFDSystem<RBDDynamics, ACTUATOR_STATE_DIM, EE_ARE_CONTROL_INPUTS>* clone() const override
    {
        return new FixBaseFDSystem<RBDDynamics, ACTUATOR_STATE_DIM, EE_ARE_CONTROL_INPUTS>(*this);
    }

    //! transform control systems state vector to a RBDState
    typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
    {
        typename RBDDynamics::RBDState_t x;
        x.setZero();
        x.basePose() = basePose_;
        x.joints().getPositions() = state.template segment<RBDDynamics::NJOINTS>(0);
        x.joints().getVelocities() = state.template segment<RBDDynamics::NJOINTS>(STATE_DIM / 2);
        return x;
    }

    //! transform control systems state vector to the pure actuator state
    core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actuatorStateFromVector(
        const core::StateVector<STATE_DIM, SCALAR>& state)
    {
        core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actState;
        actState.template topRows<ACTUATOR_STATE_DIM / 2>() =
            state.template segment<ACTUATOR_STATE_DIM / 2>(RBDDynamics::NSTATE / 2);
        actState.template bottomRows<ACTUATOR_STATE_DIM / 2>() = state.template bottomRows<ACTUATOR_STATE_DIM / 2>();
        return actState;
    }

private:
    //! a "dummy" base pose, which is always identity/zero for the fix-base case
    tpl::RigidBodyPose<SCALAR> basePose_;

    RBDDynamics dynamics_;

    std::shared_ptr<ActuatorDynamics<RBDDynamics::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>> actuatorDynamics_;
};

}  // namespace rbd
}  // namespace ct

#undef ACTUATOR_DYNAMICS_ENABLED
#undef ACTUATOR_DYNAMICS_DISABLED
