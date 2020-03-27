/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, size_t CONTROL_DIM>
SubstepRecorder<MANIFOLD, CONTROL_DIM>::SubstepRecorder(std::shared_ptr<ControlledSystem_t> system,
    bool activated,
    std::shared_ptr<ManifoldArray_t> states,
    std::shared_ptr<ControlVectorArray_t> controls,
    std::shared_ptr<TimeArray_t> times)
    : activated_(activated), system_(system), states_(states), controls_(controls), times_(times)
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
SubstepRecorder<MANIFOLD, CONTROL_DIM>::~SubstepRecorder()
{
}

template <typename MANIFOLD, size_t CONTROL_DIM>
bool SubstepRecorder<MANIFOLD, CONTROL_DIM>::callOnSubsteps()
{
    return true;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void SubstepRecorder<MANIFOLD, CONTROL_DIM>::setControlledSystem(const std::shared_ptr<ControlledSystem_t>& system)
{
    system_ = system;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
bool SubstepRecorder<MANIFOLD, CONTROL_DIM>::checkEvent(const MANIFOLD& state, const SCALAR& t)
{
    return activated_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void SubstepRecorder<MANIFOLD, CONTROL_DIM>::handleEvent(const MANIFOLD& state, const SCALAR& t)
{
    states_->push_back(state);
    controls_->push_back(system_->getLastControlAction());
    times_->push_back(t);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void SubstepRecorder<MANIFOLD, CONTROL_DIM>::setEnable(bool activated)
{
    activated_ = activated;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
void SubstepRecorder<MANIFOLD, CONTROL_DIM>::reset()
{
    states_ = std::shared_ptr<ManifoldArray_t>(new ManifoldArray_t());
    controls_ = std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>(
        new ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>);
    times_ = std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>>(new ct::core::tpl::TimeArray<SCALAR>);
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto SubstepRecorder<MANIFOLD, CONTROL_DIM>::getSubstates() const -> const std::shared_ptr<ManifoldArray_t>&
{
    return states_;
}

template <typename MANIFOLD, size_t CONTROL_DIM>
auto SubstepRecorder<MANIFOLD, CONTROL_DIM>::getSubcontrols() const
    -> const std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>&
{
    return controls_;
}

}  // namespace core
}  // namespace ct
