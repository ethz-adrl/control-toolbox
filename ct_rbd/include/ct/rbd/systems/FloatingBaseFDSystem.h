/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/physics/EEContactModel.h>

#include "RBDSystem.h"

namespace ct {
namespace rbd {

/**
 * \brief A floating base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end-effector forces expressed in the world.
 */
template <class RBDDynamics, bool QUAT_INTEGRATION = false, bool EE_ARE_CONTROL_INPUTS = false>
class FloatingBaseFDSystem : public RBDSystem<RBDDynamics, QUAT_INTEGRATION>,
                             public core::SymplecticSystem<RBDDynamics::NSTATE / 2 + QUAT_INTEGRATION,
                                 RBDDynamics::NSTATE / 2,
                                 RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3,
                                 typename RBDDynamics::SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Dynamics = RBDDynamics;
    using Kinematics = typename RBDDynamics::Kinematics_t;

    typedef typename RBDDynamics::SCALAR SCALAR;

    const static size_t N_EE = RBDDynamics::N_EE;
    const static size_t STATE_DIM = RBDDynamics::NSTATE + QUAT_INTEGRATION;
    const static size_t CONTROL_DIM = RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * N_EE * 3;

    typedef core::StateVector<STATE_DIM, SCALAR> StateVector;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;

    typedef core::
        SymplecticSystem<RBDDynamics::NSTATE / 2 + QUAT_INTEGRATION, RBDDynamics::NSTATE / 2, CONTROL_DIM, SCALAR>
            Base;

    using ContactModel = ct::rbd::EEContactModel<Kinematics>;

    FloatingBaseFDSystem() : Base(), dynamics_(), eeContactModel_(nullptr) {}
    FloatingBaseFDSystem(const FloatingBaseFDSystem<RBDDynamics, QUAT_INTEGRATION, EE_ARE_CONTROL_INPUTS>& other)
        : Base(other), dynamics_(other.dynamics_), eeContactModel_(other.eeContactModel_->clone())
    {
    }

    virtual ~FloatingBaseFDSystem() {}
    virtual FloatingBaseFDSystem<RBDDynamics, QUAT_INTEGRATION, EE_ARE_CONTROL_INPUTS>* clone() const override
    {
        return new FloatingBaseFDSystem<RBDDynamics, QUAT_INTEGRATION, EE_ARE_CONTROL_INPUTS>(*this);
    }

    virtual RBDDynamics& dynamics() override { return dynamics_; }
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    void setContactModel(const std::shared_ptr<ContactModel>& contactModel) { eeContactModel_ = contactModel; }
    virtual void computePdot(const StateVector& x,
        const core::StateVector<RBDDynamics::NSTATE / 2, SCALAR>& v,
        const ControlVector& control,
        core::StateVector<RBDDynamics::NSTATE / 2 + QUAT_INTEGRATION, SCALAR>& pDot) override
    {
        StateVector xLocal = x;

        xLocal.tail(RBDDynamics::NSTATE / 2) = v;

        typename RBDDynamics::RBDState_t rbdCached = RBDStateFromVector(xLocal);

        typename RBDDynamics::RBDAcceleration_t xd;

        pDot = toStateDerivative<QUAT_INTEGRATION>(xd, rbdCached).head(RBDDynamics::NSTATE / 2 + QUAT_INTEGRATION);
    }

    virtual void computeVdot(const StateVector& x,
        const core::StateVector<RBDDynamics::NSTATE / 2 + QUAT_INTEGRATION, SCALAR>& p,
        const ControlVector& control,
        core::StateVector<RBDDynamics::NSTATE / 2, SCALAR>& vDot) override
    {
        StateVector xLocal = x;
        xLocal.head(RBDDynamics::NSTATE / 2 + QUAT_INTEGRATION) = p;

        // Cache updated rbd state
        typename RBDDynamics::RBDState_t rbdCached = RBDStateFromVector(xLocal);
        typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

        std::array<typename Kinematics::EEForceLinear, N_EE> eeForcesW;
        eeForcesW.fill(Kinematics::EEForceLinear::Zero());

        if (eeContactModel_)
            eeForcesW = eeContactModel_->computeContactForces(rbdCached);

        if (EE_ARE_CONTROL_INPUTS)
            for (size_t i = 0; i < N_EE; i++)
                eeForcesW[i] += control.template segment<3>(RBDDynamics::NJOINTS + i * 3);

        mapEndeffectorForcesToLinkForces(rbdCached, eeForcesW, linkForces);

        typename RBDDynamics::RBDAcceleration_t xd;

        dynamics_.FloatingBaseForwardDynamics(rbdCached, control.template head<RBDDynamics::NJOINTS>(), linkForces, xd);

        vDot = toStateDerivative<QUAT_INTEGRATION>(xd, rbdCached).tail(RBDDynamics::NSTATE / 2);
    }

    /**
	 * Maps the end-effector forces expressed in the world to the link frame as required by robcogen.
	 * The link forces are transformed from world frame to the link frame. The according momentum is added.
	 * @param state robot state
	 * @param control end-effector forces expressed in the world
	 * @param linkForces forces acting on the link expressed in the link frame
	 */
    void mapEndeffectorForcesToLinkForces(const typename RBDDynamics::RBDState_t& state,
        const std::array<typename Kinematics::EEForceLinear, N_EE>& eeForcesW,
        typename RBDDynamics::ExtLinkForces_t& linkForces)
    {
        for (size_t i = 0; i < N_EE; i++)
        {
            auto endEffector = dynamics_.kinematics().getEndEffector(i);
            size_t linkId = endEffector.getLinkId();
            linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
                dynamics_.kinematics().mapForceFromWorldToLink3d(
                    eeForcesW[i], state.basePose(), state.jointPositions(), i);
        }
    }

    typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
    {
        return RBDStateFromVectorImpl<QUAT_INTEGRATION>(state);
    }

    template <bool T>
    typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state,
        typename std::enable_if<T, bool>::type = true)
    {
        typename RBDDynamics::RBDState_t x(tpl::RigidBodyPose<SCALAR>::QUAT);
        x.fromStateVectorQuaternion(state);
        return x;
    }

    template <bool T>
    typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        typename RBDDynamics::RBDState_t x(tpl::RigidBodyPose<SCALAR>::EULER);
        x.fromStateVectorEulerXyz(state);
        return x;
    }

    template <bool T>
    core::StateVector<STATE_DIM, SCALAR> toStateDerivative(const typename RBDDynamics::RBDAcceleration_t& acceleration,
        const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<T, bool>::type = true)
    {
        return acceleration.toStateUpdateVectorQuaternion(state);
    }

    template <bool T>
    core::StateVector<STATE_DIM, SCALAR> toStateDerivative(const typename RBDDynamics::RBDAcceleration_t& acceleration,
        const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        return acceleration.toStateUpdateVectorEulerXyz(state);
    }

    template <bool T>
    core::StateVector<STATE_DIM, SCALAR> toStateVector(const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<T, bool>::type = true)
    {
        return state.toStateVectorQuaternion();
    }

    template <bool T>
    core::StateVector<STATE_DIM, SCALAR> toStateVector(const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        return state.toStateVectorEulerXyz();
    }

private:
    RBDDynamics dynamics_;
    std::shared_ptr<ContactModel> eeContactModel_;
};

}  // namespace rbd
}  // namespace ct
