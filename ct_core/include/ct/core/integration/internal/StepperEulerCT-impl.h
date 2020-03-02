#pragma once

namespace ct {
namespace core {
namespace internal {

template <typename MANIFOLD, typename SCALAR>
StepperEulerCT<MANIFOLD, SCALAR>::StepperEulerCT()
{
}

template <typename MANIFOLD, typename SCALAR>
StepperEulerCT<MANIFOLD, SCALAR>::~StepperEulerCT()
{
}

template <typename MANIFOLD, typename SCALAR>
void StepperEulerCT<MANIFOLD, SCALAR>::do_step(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
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