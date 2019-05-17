/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS>
WholeBodyController<NJOINTS>::WholeBodyController()
{
}

template <size_t NJOINTS>
WholeBodyController<NJOINTS>::~WholeBodyController()
{
}

template <size_t NJOINTS>
WholeBodyController<NJOINTS>* WholeBodyController<NJOINTS>::clone() const
{
    throw std::runtime_error("Not implemented");
}

template <size_t NJOINTS>
void WholeBodyController<NJOINTS>::computeControl(const core::StateVector<STATE_DIM>& state,
    const core::Time& t,
    core::ControlVector<NJOINTS>& control)
{
    ct::rbd::RBDState<NJOINTS> x;
    x.fromStateVectorEulerXyz(state);
    core::StateVector<2 * NJOINTS> jState = x.joints().toImplementation();

    jointController_.computeControl(jState, t, control);
}

template <size_t NJOINTS>
JointPositionPIDController<NJOINTS>& WholeBodyController<NJOINTS>::getJointController()
{
    return jointController_;
}

}  // namespace rbd
}  // namespace ct
