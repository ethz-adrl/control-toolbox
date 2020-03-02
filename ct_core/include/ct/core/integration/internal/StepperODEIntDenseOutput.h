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
template <class STEPPER, typename MANIFOLD, typename SCALAR>
class StepperODEIntDenseOutput : public StepperODEInt<STEPPER, MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename boost::numeric::odeint::result_of::make_dense_output<STEPPER>::type StepperDense;
    using Tangent = typename MANIFOLD::Tangent;

    StepperODEIntDenseOutput();
    virtual ~StepperODEIntDenseOutput();

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
    StepperDense stepperDense_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct
