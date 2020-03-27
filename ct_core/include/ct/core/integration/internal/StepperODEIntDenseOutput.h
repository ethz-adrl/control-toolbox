#pragma once

#include <boost/numeric/odeint.hpp>
#include "SteppersODEIntDefinitions.h"
#include "StepperODEInt.h"
#include "../manifIntegration.h"
#include "../eigenIntegration.h"

namespace ct {
namespace core {
namespace internal {

/**
 * @brief      The interface to call ODEInt Dense Output Integration routines
 *
 * @tparam     STEPPER  The ODEInt stepper
 * @tparam     MATRIX   The matrix type
 * @tparam     SCALAR   The scalar type
 */
template <class STEPPER, typename MANIFOLD>
class StepperODEIntDenseOutput : public StepperODEInt<STEPPER, MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename boost::numeric::odeint::result_of::make_dense_output<STEPPER>::type StepperDense;

    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;
    using SystemFunction_t = typename StepperODEInt<STEPPER, MANIFOLD>::SystemFunction_t;
    using ObserverFunction_t = typename StepperODEInt<STEPPER, MANIFOLD>::ObserverFunction_t;

    StepperODEIntDenseOutput();
    virtual ~StepperODEIntDenseOutput();

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
    StepperDense stepperDense_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct
