/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/systems/FixBaseFDSystem.h>

namespace ct {
namespace rbd {

template <class RBDDynamics>
class InfiniteHorizonLQRwithInverseDynamics : public ct::core::Controller<RBDDynamics::NSTATE, RBDDynamics::NJOINTS>
{
public:
    typedef typename RBDDynamics::SCALAR SCALAR;
    typedef typename RBDDynamics::control_vector_t control_vector_t;
    typedef typename RBDDynamics::state_vector_t state_vector_t;
    typedef typename RBDDynamics::JointAcceleration_t JointAcceleration_t;

    typedef core::StateMatrix<RBDDynamics::NSTATE, SCALAR> state_matrix_t;
    typedef core::StateMatrix<RBDDynamics::NJOINTS, SCALAR> control_matrix_t;
    typedef core::StateControlMatrix<RBDDynamics::NSTATE, RBDDynamics::NJOINTS, SCALAR> state_control_matrix_t;
    typedef core::FeedbackMatrix<RBDDynamics::NSTATE, RBDDynamics::NJOINTS, SCALAR> feedback_matrix_t;


    InfiniteHorizonLQRwithInverseDynamics() {}
    InfiniteHorizonLQRwithInverseDynamics(std::shared_ptr<ct::rbd::FixBaseFDSystem<RBDDynamics>> system,
        std::shared_ptr<ct::core::LinearSystem<2 * (RBDDynamics::NJOINTS), RBDDynamics::NJOINTS>> linearSystem)
        : fdSystem_(system),
          linearSystem_(linearSystem),
          infiniteHorizonLQR_(),
          stateSetpoint_(state_vector_t::Zero()),
          id_torques_(control_vector_t::Zero()),
          K_(feedback_matrix_t::Zero())
    {
    }

    InfiniteHorizonLQRwithInverseDynamics(const InfiniteHorizonLQRwithInverseDynamics& arg)
        : fdSystem_(arg.fdSystem_->clone()),
          linearSystem_(arg.linearSystem_->clone()),
          infiniteHorizonLQR_(),
          stateSetpoint_(arg.stateSetpoint_),
          id_torques_(arg.id_torques_),
          K_(arg.K_)
    {
    }


    virtual ~InfiniteHorizonLQRwithInverseDynamics() {}
    virtual InfiniteHorizonLQRwithInverseDynamics* clone() const override
    {
        return new InfiniteHorizonLQRwithInverseDynamics<RBDDynamics>(*this);
    }


    virtual void computeControl(const ct::core::StateVector<RBDDynamics::NSTATE, SCALAR>& state,
        const ct::core::Time& t,
        ct::core::ControlVector<RBDDynamics::NJOINTS, SCALAR>& controlAction) override
    {
        controlAction = id_torques_ - K_ * (state - stateSetpoint_);
    }


    const feedback_matrix_t& getK() { return K_; }
    bool designInfiniteHorizonLQR(const state_vector_t& stateSetpoint,
        const state_matrix_t& Q_stab,
        const control_matrix_t& R_stab,
        feedback_matrix_t& K)
    {
        computeIDTorques(id_torques_, stateSetpoint);

        // local linearizations
        state_matrix_t A_setpoint = linearSystem_->getDerivativeState(stateSetpoint, id_torques_);
        state_control_matrix_t B_setpoint = linearSystem_->getDerivativeControl(stateSetpoint, id_torques_);

        bool success = infiniteHorizonLQR_.compute(Q_stab, R_stab, A_setpoint, B_setpoint, K, false, true);

        // save to local class members
        stateSetpoint_ = stateSetpoint;
        K_ = K;

        return success;
    }


    /*
	 * call this design method if you don't want to get the feedback gain matrix explicitly
	 */
    bool designInfiniteHorizonLQR(const state_vector_t& stateSetpoint,
        const state_matrix_t& Q_stab,
        const control_matrix_t& R_stab)
    {
        feedback_matrix_t K_temp = feedback_matrix_t::Zero();

        return designInfiniteHorizonLQR(stateSetpoint, Q_stab, R_stab, K_temp);
    }


    const control_vector_t& getIDTorques() const { return id_torques_; }
    const state_vector_t& getStateSetpoint() const { return stateSetpoint_; }
    void setNonlinearSystem(std::shared_ptr<ct::rbd::FixBaseFDSystem<RBDDynamics>> system) { fdSystem_ = system; }
    void setLinearSystem(
        std::shared_ptr<ct::core::LinearSystem<2 * (RBDDynamics::NJOINTS), RBDDynamics::NJOINTS>> system)
    {
        linearSystem_ = system;
    }

private:
    /*
	 * compute desired joint torques through inverse dynamics
	 */
    void computeIDTorques(control_vector_t& id_torques,
        const state_vector_t& stateSetpoint,
        const JointAcceleration_t& qdd = JointAcceleration_t::Zero())
    {
        typename RBDDynamics::ExtLinkForces_t extlinkforce(Eigen::Matrix<SCALAR, 6, 1>::Zero());

        fdSystem_->dynamics().FixBaseID(stateSetpoint, qdd, extlinkforce, id_torques);
    }


    std::shared_ptr<ct::rbd::FixBaseFDSystem<RBDDynamics>> fdSystem_;
    std::shared_ptr<ct::core::LinearSystem<2 * (RBDDynamics::NJOINTS), RBDDynamics::NJOINTS>> linearSystem_;

    ct::optcon::LQR<RBDDynamics::NSTATE, RBDDynamics::NJOINTS> infiniteHorizonLQR_;

    state_vector_t stateSetpoint_;
    control_vector_t id_torques_;
    feedback_matrix_t K_;
};
}
}
