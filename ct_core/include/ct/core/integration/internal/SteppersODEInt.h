/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <boost/numeric/odeint.hpp>
#include "SteppersODEIntDefinitions.h"

namespace ct {
namespace core {
namespace internal {

/**
 * @brief      The interface to call the integration routines from ODEInt
 *
 * @tparam     STEPPER  The ODEInt stepper
 * @tparam     MATRIX   The matrix type
 * @tparam     SCALAR   The scalar type
 */
template <class STEPPER, typename MATRIX, typename SCALAR = double>
class StepperODEInt : public StepperBase<MATRIX, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperODEInt() {}
    virtual void integrate_n_steps(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override
    {
        boost::numeric::odeint::integrate_n_steps(stepper_, rhs, state, startTime, dt, numSteps);
    }

    virtual void integrate_n_steps(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override
    {
        boost::numeric::odeint::integrate_n_steps(stepper_, rhs, state, startTime, dt, numSteps, observer);
    }


    virtual void integrate_const(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt) override
    {
        boost::numeric::odeint::integrate_const(stepper_, rhs, state, startTime, finalTime, dt);
    }

    virtual void integrate_const(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt) override
    {
        boost::numeric::odeint::integrate_const(stepper_, rhs, state, startTime, finalTime, dt, observer);
    }

    virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01)) override
    {
        boost::numeric::odeint::integrate_adaptive(stepper_, rhs, state, startTime, finalTime, dtInitial);
    }

    virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        const SCALAR dtInitial = SCALAR(0.01)) override
    {
        boost::numeric::odeint::integrate_adaptive(stepper_, rhs, state, startTime, finalTime, dtInitial, observer);
    }

    virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        SCALAR dtInitial = SCALAR(0.01)) override
    {
        boost::numeric::odeint::integrate_times(
            stepper_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
    }

private:
    STEPPER stepper_;
};


/**
 * @brief      The interface to call ODEInt Dense Output Integration routines
 *
 * @tparam     STEPPER  The ODEInt stepper
 * @tparam     MATRIX   The matrix type
 * @tparam     SCALAR   The scalar type
 */
template <class STEPPER, typename MATRIX, typename SCALAR = double>
class StepperODEIntDenseOutput : public StepperODEInt<STEPPER, MATRIX, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename boost::numeric::odeint::result_of::make_dense_output<STEPPER>::type StepperDense;

    StepperODEIntDenseOutput()
    {
        stepperDense_ = boost::numeric::odeint::make_dense_output(this->absErrTol_, this->relErrTol_, stepper_);
    }

    virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01)) override
    {
        stepperDense_.initialize(state, startTime, dtInitial);
        boost::numeric::odeint::integrate_adaptive(stepperDense_, rhs, state, startTime, finalTime, dtInitial);
    }

    virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        const SCALAR dtInitial = SCALAR(0.01)) override
    {
        stepperDense_.initialize(state, startTime, dtInitial);
        boost::numeric::odeint::integrate_adaptive(
            stepperDense_, rhs, state, startTime, finalTime, dtInitial, observer);
    }

    virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        SCALAR dtInitial = SCALAR(0.01)) override
    {
        stepperDense_.initialize(state, timeTrajectory.front(), dtInitial);
        boost::numeric::odeint::integrate_times(
            stepperDense_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
    }

private:
    STEPPER stepper_;
    StepperDense stepperDense_;
};

/**
 * @brief      The interface to call ODEInt Controlled integration routines
 *
 * @tparam     STEPPER  The ODEInt stepper type
 * @tparam     MATRIX   The Matrix Type
 * @tparam     SCALAR   The Scalar
 */
template <class STEPPER, typename MATRIX, typename SCALAR = double>
class StepperODEIntControlled : public StepperODEInt<STEPPER, MATRIX, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename boost::numeric::odeint::result_of::make_controlled<STEPPER>::type StepperControlled;

    StepperODEIntControlled()
    {
        stepperControlled_ = boost::numeric::odeint::make_controlled(this->absErrTol_, this->relErrTol_, stepper_);
    }

    virtual void integrate_adaptive(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01)) override
    {
        boost::numeric::odeint::integrate_adaptive(stepperControlled_, rhs, state, startTime, finalTime, dtInitial);
    }

    virtual void integrate_adaptive(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        const SCALAR dtInitial = SCALAR(0.01)) override
    {
        boost::numeric::odeint::integrate_adaptive(
            stepperControlled_, rhs, state, startTime, finalTime, dtInitial, observer);
    }

    virtual void integrate_times(std::function<void(const MATRIX& x, const SCALAR& t)> observer,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        SCALAR dtInitial = SCALAR(0.01)) override
    {
        boost::numeric::odeint::integrate_times(
            stepperControlled_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
    }

private:
    STEPPER stepper_;
    StepperControlled stepperControlled_;
};
}
}
}
