#pragma once

namespace ct {
namespace core {
namespace internal {

template <class STEPPER, typename MANIFOLD>
StepperODEIntControlled<STEPPER, MANIFOLD>::StepperODEIntControlled()
{
    stepperControlled_ = boost::numeric::odeint::make_controlled(this->absErrTol_, this->relErrTol_, stepper_);
}

template <class STEPPER, typename MANIFOLD>
StepperODEIntControlled<STEPPER, MANIFOLD>::~StepperODEIntControlled()
{
}

template <class STEPPER, typename MANIFOLD>
void StepperODEIntControlled<STEPPER, MANIFOLD>::integrate_adaptive(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_adaptive(stepperControlled_, rhs, state, startTime, finalTime, dtInitial);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEIntControlled<STEPPER, MANIFOLD>::integrate_adaptive(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    const SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_adaptive(
        stepperControlled_, rhs, state, startTime, finalTime, dtInitial, observer);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEIntControlled<STEPPER, MANIFOLD>::integrate_times(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
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