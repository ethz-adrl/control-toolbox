/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR>
SEADynamics1d<NJOINTS, SCALAR>::SEADynamics1d(double k_spring) : k_((SCALAR)k_spring)
{
}


template <size_t NJOINTS, typename SCALAR>
SEADynamics1d<NJOINTS, SCALAR>::~SEADynamics1d()
{
}

template <size_t NJOINTS, typename SCALAR>
SEADynamics1d<NJOINTS, SCALAR>* SEADynamics1d<NJOINTS, SCALAR>::clone() const
{
    return new SEADynamics1d(*this);
}

template <size_t NJOINTS, typename SCALAR>
void SEADynamics1d<NJOINTS, SCALAR>::computeControlledDynamics(const ct::core::StateVector<NJOINTS, SCALAR>& state,
    const SCALAR& t,
    const ct::core::ControlVector<NJOINTS, SCALAR>& control,
    ct::core::StateVector<NJOINTS, SCALAR>& derivative)
{
    derivative.template head<NJOINTS>() = control.template head<NJOINTS>();
}

template <size_t NJOINTS, typename SCALAR>
core::ControlVector<NJOINTS, SCALAR> SEADynamics1d<NJOINTS, SCALAR>::computeControlOutput(
    const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
    const typename BASE::act_state_vector_t& actState)
{
    return (actState.template topRows<NJOINTS>() - robotJointState.getPositions()) * k_;
}

template <size_t NJOINTS, typename SCALAR>
ct::core::StateVector<NJOINTS, SCALAR> SEADynamics1d<NJOINTS, SCALAR>::computeStateFromOutput(
    const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& refRobotJointState,
    const core::ControlVector<NJOINTS, SCALAR>& refControl)
{
	return refRobotJointState.getPositions() + 1/k_ * refControl.toImplementation();
}

}  // namespace rbd
}  // namespace ct
