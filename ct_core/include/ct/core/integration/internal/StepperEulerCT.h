#pragma once

#include "StepperCTBase.h"

namespace ct {
namespace core {
namespace internal {

/**
 * @brief      Custom implementation of the euler stepper
 *
 * @tparam     MATRIX  The matrix type
 * @tparam     SCALAR  The scalar type
 */
template <typename MANIFOLD>
class StepperEulerCT : public StepperCTBase<MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;
    using SystemFunction_t = typename StepperCTBase<MANIFOLD>::SystemFunction_t;

    StepperEulerCT();
    virtual ~StepperEulerCT();

private:
    virtual void do_step(const SystemFunction_t& rhs,
        MANIFOLD& stateInOut,
        const SCALAR time,
        const SCALAR dt) override;

    Tangent derivative_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct