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
template <typename MANIFOLD, typename SCALAR>
class StepperEulerCT : public StepperCTBase<MANIFOLD, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Tangent = typename MANIFOLD::Tangent;

    StepperEulerCT();
    virtual ~StepperEulerCT();

private:
    virtual void do_step(const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
        MANIFOLD& stateInOut,
        const SCALAR time,
        const SCALAR dt) override;

    Tangent derivative_;
};

}  // namespace internal
}  // namespace core
}  // namespace ct