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
template <typename MANIFOLD, typename SCALAR>
class StepperCTBase : public StepperBase<MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Tangent = typename MANIFOLD::Tangent;

    StepperCTBase();
    virtual ~StepperCTBase();

    virtual void integrate_n_steps(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override;

    virtual void integrate_n_steps(std::function<void(const MANIFOLD& x, const SCALAR& t)> observe,
        const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override;

    /**
     * @brief          Implements a single step of the integration scheme
     *
     * @param[in]      rhs         The ODE
     * @param[in, out] stateInOut  The state
     * @param[in]      time        The integration time
     * @param[in]      dt          The integration timestep
     */
    virtual void do_step(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& stateInOut,
        const SCALAR time,
        const SCALAR dt) = 0;
};

}  // namespace internal
}  // namespace core
}  // namespace ct
