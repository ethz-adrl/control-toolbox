#pragma once

namespace ct {
namespace core {
namespace internal {

template <class STEPPER, typename MANIFOLD>
StepperODEIntDenseOutput<STEPPER, MANIFOLD>::StepperODEIntDenseOutput()
{
    stepperDense_ = boost::numeric::odeint::make_dense_output(this->absErrTol_, this->relErrTol_, stepper_);
}

template <class STEPPER, typename MANIFOLD>
StepperODEIntDenseOutput<STEPPER, MANIFOLD>::~StepperODEIntDenseOutput()
{
}

template <class STEPPER, typename MANIFOLD>
void StepperODEIntDenseOutput<STEPPER, MANIFOLD>::integrate_adaptive(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    stepperDense_.initialize(state, startTime, dtInitial);
    boost::numeric::odeint::integrate_adaptive(stepperDense_, rhs, state, startTime, finalTime, dtInitial);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEIntDenseOutput<STEPPER, MANIFOLD>::integrate_adaptive(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    const SCALAR dtInitial)
{
    stepperDense_.initialize(state, startTime, dtInitial);
    boost::numeric::odeint::integrate_adaptive(stepperDense_, rhs, state, startTime, finalTime, dtInitial, observer);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEIntDenseOutput<STEPPER, MANIFOLD>::integrate_times(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const tpl::TimeArray<SCALAR>& timeTrajectory,
    SCALAR dtInitial)
{
    stepperDense_.initialize(state, timeTrajectory.front(), dtInitial);
    boost::numeric::odeint::integrate_times(
        stepperDense_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
}

}  // namespace internal
}  // namespace core
}  // namespace ct
