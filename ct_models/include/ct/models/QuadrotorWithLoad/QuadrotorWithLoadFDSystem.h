/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>
#include <ct/rbd/rbd.h>

namespace ct {
namespace rbd {

template <class RBDDynamics, bool QUAT_INTEGRATION = false>
class QuadrotorWithLoadFDSystem
    : public RBDSystem<RBDDynamics, QUAT_INTEGRATION>,
      public core::ControlledSystem<RBDDynamics::NSTATE + QUAT_INTEGRATION, 4, typename RBDDynamics::SCALAR>
{
public:
    using Dynamics = RBDDynamics;
    using Kinematics = typename RBDDynamics::Kinematics_t;

    typedef typename RBDDynamics::SCALAR SCALAR;

    const static size_t STATE_DIM = RBDDynamics::NSTATE + QUAT_INTEGRATION;
    const static size_t CONTROL_DIM = 4;

    typedef core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    QuadrotorWithLoadFDSystem(){};

    QuadrotorWithLoadFDSystem(const QuadrotorWithLoadFDSystem& arg)
        : RBDSystem<RBDDynamics, QUAT_INTEGRATION>(arg),
          core::ControlledSystem<RBDDynamics::NSTATE + QUAT_INTEGRATION, 4, typename RBDDynamics::SCALAR>(arg),
          dynamics_(arg.dynamics_)
    {
    }

    virtual ~QuadrotorWithLoadFDSystem(){};

    virtual RBDDynamics& dynamics() override { return dynamics_; }
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    void computeControlledDynamics(const core::StateVector<STATE_DIM, SCALAR>& state,
        const core::Time& t,
        const core::ControlVector<CONTROL_DIM, SCALAR>& control,
        core::StateVector<STATE_DIM, SCALAR>& derivative

        ) override
    {
        typename RBDDynamics::RBDState_t x = RBDStateFromVector(state);

        typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

        mapControlInputsToLinkForces(x, control, linkForces);

        typename RBDDynamics::RBDAcceleration_t xd;
        ct::core::ControlVector<RBDDynamics::NJOINTS> joint_torques =
            ct::core::ControlVector<RBDDynamics::NJOINTS>::Zero();

        // introduce some light damping into the joint (friction) //! @todo tune this value
        joint_torques = -0.0005 * x.joints().getVelocities();

        dynamics_.FloatingBaseForwardDynamics(x, joint_torques, linkForces, xd);

        derivative = toStateDerivative<QUAT_INTEGRATION>(xd, x);
    }


    void mapControlInputsToLinkForces(const typename RBDDynamics::RBDState_t& state,
        const core::ControlVector<CONTROL_DIM, SCALAR>& control,
        typename RBDDynamics::ExtLinkForces_t& linkForces)
    {
        /* u = [overall thrust, mx, my, mz]
		 * we directly map the control inputs into quadrotor body forces
		 * */

        size_t baseLinkId = 0;
        linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(baseLinkId)].setZero();
        linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(baseLinkId)](5) =
            control(0);  // the thrust in quadrotor z-direction
        linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(baseLinkId)].template segment<3>(0) =
            control.template segment<3>(1);  // the torques
    }

    typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
    {
        return RBDStateFromVectorImpl<QUAT_INTEGRATION>(state);
    }

    template <bool T>
    typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state,
        typename std::enable_if<T, bool>::type = true)
    {
        typename RBDDynamics::RBDState_t x(RigidBodyPose::QUAT);
        x.fromStateVectorQuaternion(state);
        return x;
    }

    template <bool T>
    typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        typename RBDDynamics::RBDState_t x(RigidBodyPose::EULER);
        x.fromStateVectorEulerXyz(state);
        return x;
    }

    template <bool T>
    core::StateVector<STATE_DIM> toStateDerivative(const typename RBDDynamics::RBDAcceleration_t& acceleration,
        const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<T, bool>::type = true)
    {
        return acceleration.toStateUpdateVectorQuaternion(state);
    }

    template <bool T>
    core::StateVector<STATE_DIM> toStateDerivative(const typename RBDDynamics::RBDAcceleration_t& acceleration,
        const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        return acceleration.toStateUpdateVectorEulerXyz(state);
    }


    virtual QuadrotorWithLoadFDSystem<RBDDynamics, QUAT_INTEGRATION>* clone() const override
    {
        return new QuadrotorWithLoadFDSystem(*this);
    }

private:
    RBDDynamics dynamics_;
};
}
}
