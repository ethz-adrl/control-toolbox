#pragma once

#include <boost/numeric/odeint.hpp>
#include "SteppersODEIntDefinitions.h"
#include "../manifIntegration.h"
#include "../eigenIntegration.h"

namespace ct {
namespace core {
namespace internal {


/**
 * @brief      The interface to call ODEInt Controlled integration routines
 *
 * @tparam     STEPPER  The ODEInt stepper type
 * @tparam     MATRIX   The Matrix Type
 * @tparam     SCALAR   The Scalar
 */
template <class STEPPER, typename MANIFOLD, typename SCALAR>
class StepperODEIntControlled : public StepperODEInt<STEPPER, MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename boost::numeric::odeint::result_of::make_controlled<STEPPER>::type StepperControlled;
    using Tangent = typename MANIFOLD::Tangent;

    StepperODEIntControlled();
    virtual ~StepperODEIntControlled();

    virtual void integrate_adaptive(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        SCALAR dtInitial = SCALAR(0.01)) override;

    virtual void integrate_adaptive(std::function<void(const MANIFOLD& x, const SCALAR& t)> observer,
        const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& state,
        const SCALAR& startTime,
        const SCALAR& finalTime,
        const SCALAR dtInitial = SCALAR(0.01)) override;

    virtual void integrate_times(std::function<void(const MANIFOLD& x, const SCALAR& t)> observer,
        const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& state,
        const tpl::TimeArray<SCALAR>& timeTrajectory,
        SCALAR dtInitial = SCALAR(0.01)) override;

private:
    STEPPER stepper_;
    StepperControlled stepperControlled_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct