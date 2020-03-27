/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <boost/numeric/odeint.hpp>
#include "StepperBase.h"

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
template <class STEPPER, typename MANIFOLD>
class StepperODEInt : public StepperBase<MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;
    using SystemFunction_t = typename StepperBase<MANIFOLD>::SystemFunction_t;
    using ObserverFunction_t = typename StepperBase<MANIFOLD>::ObserverFunction_t;

    StepperODEInt();
    virtual ~StepperODEInt();

    virtual void integrate_n_steps(const SystemFunction_t& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override;

    virtual void integrate_n_steps(ObserverFunction_t observer,
        const SystemFunction_t& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        size_t numSteps,
        SCALAR dt) override;

    virtual void integrate_const(const SystemFunction_t& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt) override;

    virtual void integrate_const(ObserverFunction_t observer,
        const SystemFunction_t& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dt) override;

    virtual void integrate_adaptive(const SystemFunction_t& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01)) override;

    virtual void integrate_adaptive(ObserverFunction_t observer,
        const SystemFunction_t& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        const SCALAR dtInitial = SCALAR(0.01)) override;

    virtual void integrate_times(ObserverFunction_t observer,
        const SystemFunction_t& rhs,
        MANIFOLD& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        SCALAR dtInitial = SCALAR(0.01)) override;

private:
    STEPPER stepper_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct
