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
template <typename MANIFOLD, typename SCALAR = double>
class StepperRK4CT : public StepperCTBase<MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Tangent = typename MANIFOLD::Tangent;

    StepperRK4CT();
    virtual ~StepperRK4CT();

private:
    virtual void do_step(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
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