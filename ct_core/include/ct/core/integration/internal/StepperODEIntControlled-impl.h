#pragma once

namespace ct {
namespace core {
namespace internal {

template <class STEPPER, typename MANIFOLD, typename SCALAR>
StepperODEIntControlled<STEPPER, MANIFOLD, SCALAR>::StepperODEIntControlled()
{
    stepperControlled_ = boost::numeric::odeint::make_controlled(this->absErrTol_, this->relErrTol_, stepper_);
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
StepperODEIntControlled<STEPPER, MANIFOLD, SCALAR>::~StepperODEIntControlled()
{
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
void StepperODEIntControlled<STEPPER, MANIFOLD, SCALAR>::integrate_adaptive(
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_adaptive(stepperControlled_, rhs, state, startTime, finalTime, dtInitial);
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
void StepperODEIntControlled<STEPPER, MANIFOLD, SCALAR>::integrate_adaptive(
    std::function<void(const MANIFOLD& x, const SCALAR& t)> observer,
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    const SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_adaptive(
        stepperControlled_, rhs, state, startTime, finalTime, dtInitial, observer);
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
void StepperODEIntControlled<STEPPER, MANIFOLD, SCALAR>::integrate_times(
    std::function<void(const MANIFOLD& x, const SCALAR& t)> observer,
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const tpl::TimeArray<SCALAR>& timeTrajectory,
    SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_times(
        stepperControlled_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
}

}  // namespace internal
}  // namespace core
}  // namespace ct