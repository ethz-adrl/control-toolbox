/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


#include <ct/core/systems/ControlledSystem.h>
#include <ct/rbd/rbd.h>

#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/physics/EEContactModel.h>

#include "HyQForwardZero.h"

namespace ct {
namespace models {
namespace HyQ {

/**
 * \brief A floating base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end-effector forces expressed in the world.
 */
class HyQContactModelForwardZeroSystem : public core::SymplecticSystem<18, 18, 12>
{
public:
    const static size_t STATE_DIM = 36;
    const static size_t CONTROL_DIM = 12;

    typedef core::StateVector<STATE_DIM> StateVector;
    typedef core::ControlVector<CONTROL_DIM> ControlVector;

    typedef core::SymplecticSystem<18, 18, CONTROL_DIM> Base;

    HyQContactModelForwardZeroSystem() : Base(){};

    HyQContactModelForwardZeroSystem(const HyQContactModelForwardZeroSystem& other)
        : Base(other), hyqForwardZero_(other.hyqForwardZero_){};

    virtual ~HyQContactModelForwardZeroSystem(){};

    virtual HyQContactModelForwardZeroSystem* clone() const override
    {
        return new HyQContactModelForwardZeroSystem(*this);
    }

    virtual void computePdot(const StateVector& x,
        const core::StateVector<18>& v,
        const ControlVector& control,
        core::StateVector<18>& pDot) override
    {
        StateVector xLocal = x;
        xLocal.tail(18) = v;
        ct::rbd::RBDState<12> rbdCached = RBDStateFromVector(xLocal);
        ct::rbd::RBDAcceleration<12> xd;
        pDot = toStateDerivative(xd, rbdCached).head(18);
    }

    virtual void computeVdot(const StateVector& x,
        const core::StateVector<18>& p,
        const ControlVector& control,
        core::StateVector<18>& vDot) override
    {
        StateVector xLocal = x;
        xLocal.head(18) = p;
        Eigen::Matrix<double, STATE_DIM + CONTROL_DIM + 1, 1> xut;
        xut << xLocal, control, 0.0;
        vDot = hyqForwardZero_.forwardZero(xut).tail(18);
    }


private:
    HyQForwardZero hyqForwardZero_;

    StateVector toStateDerivative(const ct::rbd::RBDAcceleration<12>& acceleration, const ct::rbd::RBDState<12>& state)
    {
        return acceleration.toStateUpdateVectorEulerXyz(state);
    }

    ct::rbd::RBDState<12> RBDStateFromVector(const StateVector& state)
    {
        ct::rbd::RBDState<12> x;
        x.fromStateVectorEulerXyz(state);
        return x;
    }
};

}  // namespace HyQ

}  // namespace models
}  // namespace ct
