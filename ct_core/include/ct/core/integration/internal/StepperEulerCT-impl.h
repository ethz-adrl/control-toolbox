#pragma once

namespace ct {
namespace core {
namespace internal {

template <typename MANIFOLD>
StepperEulerCT<MANIFOLD>::StepperEulerCT()
{
}

template <typename MANIFOLD>
StepperEulerCT<MANIFOLD>::~StepperEulerCT()
{
}

template <typename MANIFOLD>
void StepperEulerCT<MANIFOLD>::do_step(const SystemFunction_t& rhs,
    MANIFOLD& stateInOut,
    const SCALAR time,
    const SCALAR dt)
{
    rhs(stateInOut, derivative_, time);

    // TODO: fails if order of multiplication is other way round ... why? Is this intentional?
    stateInOut = stateInOut + derivative_ * dt;
}

}  // namespace internal
}  // namespace core
}  // namespace ct