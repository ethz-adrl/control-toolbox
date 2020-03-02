/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::SubstepRecorder(
    std::shared_ptr<ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>> system,
    bool activated,
    std::shared_ptr<ManifoldArray_t> states,
    std::shared_ptr<ControlVectorArray_t> controls,
    std::shared_ptr<TimeArray_t> times)
    : activated_(activated), system_(system), states_(states), controls_(controls), times_(times)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::~SubstepRecorder()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
bool SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::callOnSubsteps()
{
    return true;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::setControlledSystem(
    const std::shared_ptr<ControlledSystem<MANIFOLD, CONTROL_DIM, SCALAR>>& system)
{
    system_ = system;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
bool SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::checkEvent(const MANIFOLD& state, const SCALAR& t)
{
    return activated_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::handleEvent(const MANIFOLD& state, const SCALAR& t)
{
    states_->push_back(state);
    controls_->push_back(system_->getLastControlAction());
    times_->push_back(t);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::setEnable(bool activated)
{
    activated_ = activated;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
void SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::reset()
{
    states_ = std::shared_ptr<ManifoldArray_t>(new ManifoldArray_t());
    controls_ = std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>(
        new ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>);
    times_ = std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>>(new ct::core::tpl::TimeArray<SCALAR>);
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
auto SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::getSubstates() const -> const std::shared_ptr<ManifoldArray_t>&
{
    return states_;
}

template <typename MANIFOLD, size_t CONTROL_DIM, typename SCALAR>
const std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>&
SubstepRecorder<MANIFOLD, CONTROL_DIM, SCALAR>::getSubcontrols() const
{
    return controls_;
}

}  // namespace core
}  // namespace ct
