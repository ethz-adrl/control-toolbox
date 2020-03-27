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
template <class STEPPER, typename MANIFOLD>
class StepperODEIntControlled : public StepperODEInt<STEPPER, MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename boost::numeric::odeint::result_of::make_controlled<STEPPER>::type StepperControlled;
    
        using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;
    using SystemFunction_t = typename StepperODEInt<STEPPER, MANIFOLD>::SystemFunction_t;
    using ObserverFunction_t = typename StepperODEInt<STEPPER, MANIFOLD>::ObserverFunction_t;

    StepperODEIntControlled();
    virtual ~StepperODEIntControlled();

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
    StepperControlled stepperControlled_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct