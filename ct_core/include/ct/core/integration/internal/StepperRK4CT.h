#pragma once

#include "StepperCTBase.h"

namespace ct {
namespace core {
namespace internal {

/**
 * @brief      Custom implementation of the rk4 integration scheme
 *
 * @tparam     MATRIX  The matrix type
 * @tparam     SCALAR  The scalar type
 */
template <typename MANIFOLD>
class StepperRK4CT : public StepperCTBase<MANIFOLD>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = typename MANIFOLD::Scalar;
    using Tangent = typename MANIFOLD::Tangent;
    using SystemFunction_t = typename StepperCTBase<MANIFOLD>::SystemFunction_t;

    StepperRK4CT();
    virtual ~StepperRK4CT();

private:
    virtual void do_step(const SystemFunction_t& rhs,
        MANIFOLD& stateInOut,
        const SCALAR time,
        const SCALAR dt) override;

    Tangent k1_;
    Tangent k2_;
    Tangent k3_;
    Tangent k4_;
    SCALAR oneSixth_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct