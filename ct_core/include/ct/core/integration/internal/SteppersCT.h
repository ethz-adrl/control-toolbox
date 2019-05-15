/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/systems/continuous_time/System.h>
#include "StepperBase.h"

namespace ct {
namespace core {
namespace internal {


/**
 * @brief      The stepper interface for custom steppers
 *
 * @tparam     MATRIX  The Matrix type to be integrated
 * @tparam     SCALAR  The scalar type
 */
template <typename MATRIX, typename SCALAR = double>
class StepperCTBase : public StepperBase<MATRIX, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void integrate_n_steps(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override
    {
        SCALAR time = startTime;
        for (size_t i = 0; i < numSteps; ++i)
        {
            do_step(rhs, state, time, dt);
            time += dt;
        }
    }

    virtual void integrate_n_steps(std::function<void(const MATRIX& x, const SCALAR& t)> observe,
        const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override
    {
        SCALAR time = startTime;

        for (size_t i = 0; i < numSteps; ++i)
        {
            do_step(rhs, state, time, dt);
            time += dt;
            observe(state, time);
        }
    }

    /**
     * @brief          Implements a single step of the integration scheme
     *
     * @param[in]      rhs         The ODE
     * @param[in, out] stateInOut  The state
     * @param[in]      time        The integration time
     * @param[in]      dt          The integration timestep
     */
    virtual void do_step(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& stateInOut,
        const SCALAR time,
        const SCALAR dt) = 0;
};


/**
 * @brief      Custom implementation of the euler stepper
 *
 * @tparam     MATRIX  The matrix type
 * @tparam     SCALAR  The scalar type
 */
template <typename MATRIX, typename SCALAR = double>
class StepperEulerCT : public StepperCTBase<MATRIX, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperEulerCT() {}
private:
    virtual void do_step(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& stateInOut,
        const SCALAR time,
        const SCALAR dt) override
    {
        rhs(stateInOut, derivative_, time);
        stateInOut += dt * derivative_;
    }

    MATRIX derivative_;
};

/**
 * @brief      Custom implementation of the rk4 integration scheme
 *
 * @tparam     MATRIX  The matrix type
 * @tparam     SCALAR  The scalar type
 */
template <typename MATRIX, typename SCALAR = double>
class StepperRK4CT : public StepperCTBase<MATRIX, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StepperRK4CT() : oneSixth_(SCALAR(1.0 / 6.0)) {}
private:
    virtual void do_step(const std::function<void(const MATRIX&, MATRIX&, SCALAR)>& rhs,
        MATRIX& stateInOut,
        const SCALAR time,
        const SCALAR dt) override
    {
        SCALAR halfStep = SCALAR(0.5) * dt;
        SCALAR timePlusHalfStep = time + halfStep;
        rhs(stateInOut, k1_, time);
        rhs(stateInOut + halfStep * k1_, k2_, timePlusHalfStep);
        rhs(stateInOut + halfStep * k2_, k3_, timePlusHalfStep);
        rhs(stateInOut + dt * k3_, k4_, time + dt);
        stateInOut += oneSixth_ * dt * (k1_ + SCALAR(2.0) * k2_ + SCALAR(2.0) * k3_ + k4_);
    }

    MATRIX k1_;
    MATRIX k2_;
    MATRIX k3_;
    MATRIX k4_;
    SCALAR oneSixth_;
};
}
}
}
