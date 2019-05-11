/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR>
SEADynamicsFirstOrder<NJOINTS, SCALAR>::SEADynamicsFirstOrder(double k_spring, double gear_ratio)
    : k_(SCALAR(k_spring)), r_(SCALAR(gear_ratio))
{
}

template <size_t NJOINTS, typename SCALAR>
SEADynamicsFirstOrder<NJOINTS, SCALAR>::~SEADynamicsFirstOrder()
{
}

template <size_t NJOINTS, typename SCALAR>
SEADynamicsFirstOrder<NJOINTS, SCALAR>* SEADynamicsFirstOrder<NJOINTS, SCALAR>::clone() const
{
    return new SEADynamicsFirstOrder(*this);
}

template <size_t NJOINTS, typename SCALAR>
void SEADynamicsFirstOrder<NJOINTS, SCALAR>::computeActuatorDynamics(const JointState<NJOINTS, SCALAR>& robotJointState,
    const ct::core::StateVector<NJOINTS, SCALAR>& state,
    const SCALAR& t,
    const ct::core::ControlVector<NJOINTS, SCALAR>& control,
    ct::core::StateVector<NJOINTS, SCALAR>& derivative)
{
    // the gear velocity is the motor velocity divided by the gear ratio:
    derivative.template head<NJOINTS>() = control.template head<NJOINTS>() / r_;
}

template <size_t NJOINTS, typename SCALAR>
core::ControlVector<NJOINTS, SCALAR> SEADynamicsFirstOrder<NJOINTS, SCALAR>::computeControlOutput(
    const JointState<NJOINTS, SCALAR>& robotJointState,
    const typename BASE::act_state_vector_t& actState)
{
    // compute joint torque as a function of the deflection between joint position and gear position
    return (actState.template topRows<NJOINTS>() - robotJointState.getPositions()) * k_;
}

template <size_t NJOINTS, typename SCALAR>
ct::core::StateVector<NJOINTS, SCALAR> SEADynamicsFirstOrder<NJOINTS, SCALAR>::computeStateFromOutput(
    const JointState<NJOINTS, SCALAR>& refRobotJointState,
    const core::ControlVector<NJOINTS, SCALAR>& refControl)
{
    // compute desired gear position from current joint position and desired torque
    return refRobotJointState.getPositions() + refControl.toImplementation() / k_;
}

}  // namespace rbd
}  // namespace ct
