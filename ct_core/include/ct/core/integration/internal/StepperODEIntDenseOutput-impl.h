#pragma once

namespace ct {
namespace core {
namespace internal {

template <class STEPPER, typename MANIFOLD, typename SCALAR>
StepperODEIntDenseOutput<STEPPER, MANIFOLD, SCALAR>::StepperODEIntDenseOutput()
{
    stepperDense_ = boost::numeric::odeint::make_dense_output(this->absErrTol_, this->relErrTol_, stepper_);
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
StepperODEIntDenseOutput<STEPPER, MANIFOLD, SCALAR>::~StepperODEIntDenseOutput()
{
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
void StepperODEIntDenseOutput<STEPPER, MANIFOLD, SCALAR>::integrate_adaptive(
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    stepperDense_.initialize(state, startTime, dtInitial);
    boost::numeric::odeint::integrate_adaptive(stepperDense_, rhs, state, startTime, finalTime, dtInitial);
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
void StepperODEIntDenseOutput<STEPPER, MANIFOLD, SCALAR>::integrate_adaptive(
    std::function<void(const MANIFOLD& x, const SCALAR& t)> observer,
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    const SCALAR dtInitial)
{
    stepperDense_.initialize(state, startTime, dtInitial);
    boost::numeric::odeint::integrate_adaptive(stepperDense_, rhs, state, startTime, finalTime, dtInitial, observer);
}

template <class STEPPER, typename MANIFOLD, typename SCALAR>
void StepperODEIntDenseOutput<STEPPER, MANIFOLD, SCALAR>::integrate_times(
    std::function<void(const MANIFOLD& x, const SCALAR& t)> observer,
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
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
