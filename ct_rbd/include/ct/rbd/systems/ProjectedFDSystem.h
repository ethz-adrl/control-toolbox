
#pragma once

#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/physics/EEContactModel.h>

#include "RBDSystem.h"

namespace ct {
namespace rbd {

/*!
 * \brief A floating base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end-effector forces expressed in the world.
 *
 * This class creates a ct::core::ControlledSystem based on ProjectedDynamics to make it
 * compatible with ct::core functionality such as a ct::core::Integrator. Please note that
 * there are multiple choices of transforming a Rigid Body Dynamics system into an ODE used
 * for integration and (optimal) control. This particular modelling is explained below. For
 * other choices see FloatingBaseFDSystem and FixBaseFDSystem.
 *
 * The goal of this class is to transform a Rigid Body Dynamics system into an ODE of the form
 * \f$ \dot{x} = f(x,u) \f$ using projected forward dynamics as implemented in ProjectedDynamics.
 * To do so, we define the state as
 * \f[
 * 	x = [ {}_W q_B ~ {}_W p_B ~ \theta_J ~ {}_B \omega_B ~ {}_B v_B ~ \dot{\theta}_J ]^T
 * \f]
 *
 * where \f$ {}_W q_B \f$ is the base orientation and \f$ {}_W p_B \f$ is the base position,
 * both expressed the world frame. \f$ {}_B \omega_B \f$ is the local angular velocity of the
 * base and \f$ {}_B v_B \f$ is the linear velocity of the base, both are expressed in the local
 * base coordinate frame. Hence, the velocities are \b NOT direct derivatives of the position/orientation.
 * \f$ \theta_J \f$ and \f$ \dot{theta}_J \f$ are the joint angles
 * and joint velocities, respectively.
 *
 * The input vector \f$ u = \tau \f$ contains the joint torques. We further assume the base is unactuated.
 *
 * Given the Projected Rigid Body Dynamics
 * \f[
 * {}_B \ddot{q}_c = P M^{-1} (J^T_c \lambda + S^T \tau - G - C)
 * \f]
 *
 * where \f$ {}_B \ddot{q}_c = [ {}_B \dot{\omega}_B ~ {}_B \dot{v}_B ~ \ddot{\theta}_J ]^T \f$
 * are the generalized coordinates expressed in the base frame.
 *
 * The system dynamics then become:
 *
 * \f[
 * \begin{aligned}
 * 	\dot{x} &= [ {}_W \dot{q}_B ~ {}_W \dot{p}_B ~ \dot{\theta}_J ~ {}_B \dot{\omega}_B ~ {}_B \dot{v}_B ~ \ddot{\theta}_J ]^T \\
 *          &= [ H_{WB} {}_B \omega_B ~ R_{WB}  {}_B v_B ~ \dot{\theta}_J ~ P M^{-1} (J^T_c \lambda + S^T \tau - G - C)]^T
 * \end{aligned}
 * \f]
 */
template <class RBDDynamics, bool QUAT_INTEGRATION = false>
class ProjectedFDSystem : public RBDSystem<RBDDynamics, QUAT_INTEGRATION>,
                          public core::ControlledSystem<RBDDynamics::NSTATE + QUAT_INTEGRATION,
                              RBDDynamics::NJOINTS,
                              typename RBDDynamics::SCALAR>
{
public:
    using Dynamics = RBDDynamics;
    using Kinematics = typename RBDDynamics::Kinematics_t;

    typedef typename RBDDynamics::SCALAR SCALAR;

    const static size_t N_EE = RBDDynamics::N_EE;
    const static size_t STATE_DIM = RBDDynamics::NSTATE + QUAT_INTEGRATION;
    const static size_t CONTROL_DIM = RBDDynamics::NJOINTS;

    typedef core::StateVector<STATE_DIM, SCALAR> StateVector;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;

    typedef core::ControlledSystem<RBDDynamics::RBDState_t::NSTATE + QUAT_INTEGRATION, RBDDynamics::NJOINTS> Base;

    ProjectedFDSystem(typename RBDDynamics::EE_in_contact_t eeInContact = true) : eeInContact_(eeInContact){};

    virtual ~ProjectedFDSystem(){};

    virtual RBDDynamics& dynamics() override { return dynamics_; }
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    virtual void computeControlledDynamics(const StateVector& state,
        const SCALAR& t,
        const ControlVector& control,
        StateVector& derivative) override
    {
        typename RBDDynamics::RBDState_t x = RBDStateFromVector(state);
        typename RBDDynamics::RBDAcceleration_t xd;

        dynamics_.ProjectedForwardDynamics(eeInContact_, x, control, xd);
        derivative = toStateDerivative<QUAT_INTEGRATION>(xd, x);
    }

    void setEEInContact(typename RBDDynamics::EE_in_contact_t& eeInContact) { eeInContact_ = eeInContact; }
    typename RBDDynamics::RBDState_t RBDStateFromVector(const StateVector& state)
    {
        return RBDStateFromVectorImpl<QUAT_INTEGRATION>(state);
    }

    template <bool T>
    typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const StateVector& state,
        typename std::enable_if<T, bool>::type = true)
    {
        typename RBDDynamics::RBDState_t x(tpl::RigidBodyPose<SCALAR>::QUAT);
        x.fromStateVectorQuaternion(state);
        return x;
    }

    template <bool T>
    typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const StateVector& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        typename RBDDynamics::RBDState_t x(tpl::RigidBodyPose<SCALAR>::EULER);
        x.fromStateVectorEulerXyz(state);
        return x;
    }

    template <bool T>
    StateVector toStateDerivative(const typename RBDDynamics::RBDAcceleration_t& acceleration,
        const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<T, bool>::type = true)
    {
        return acceleration.toStateUpdateVectorQuaternion(state);
    }

    template <bool T>
    StateVector toStateDerivative(const typename RBDDynamics::RBDAcceleration_t& acceleration,
        const typename RBDDynamics::RBDState_t& state,
        typename std::enable_if<!T, bool>::type = true)
    {
        return acceleration.toStateUpdateVectorEulerXyz(state);
    }


    virtual ProjectedFDSystem<RBDDynamics, QUAT_INTEGRATION>* clone() const override
    {
        throw std::runtime_error("clone not implemented");
    }

private:
    typename RBDDynamics::EE_in_contact_t eeInContact_;

    RBDDynamics dynamics_;
};

}  // namespace rbd
}  // namespace ct
