#pragma once

namespace ct {
namespace core {
namespace internal {

template <typename MANIFOLD, typename SCALAR>
StepperRK4CT<MANIFOLD, SCALAR>::StepperRK4CT() : oneSixth_(SCALAR(1.0 / 6.0))
{
}

template <typename MANIFOLD, typename SCALAR>
StepperRK4CT<MANIFOLD, SCALAR>::~StepperRK4CT()
{
}

template <typename MANIFOLD, typename SCALAR>
void StepperRK4CT<MANIFOLD, SCALAR>::do_step(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& stateInOut,
    const SCALAR time,
    const SCALAR dt)
{
    SCALAR halfStep = SCALAR(0.5) * dt;
    SCALAR timePlusHalfStep = time + halfStep;
    rhs(stateInOut, k1_, time);
    rhs(stateInOut + k1_ * halfStep, k2_, timePlusHalfStep);
    rhs(stateInOut + k2_ * halfStep, k3_, timePlusHalfStep);
    rhs(stateInOut + k3_ * dt, k4_, time + dt);
    stateInOut = stateInOut + (k1_ + k2_ * SCALAR(2.0) + k3_ * SCALAR(2.0) + k4_) * oneSixth_ * dt;
}

}  // namespace internal
}  // namespace core
}  // namespace ct