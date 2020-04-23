#pragma once

#include <ct/core/core.h>  // always include ct-first to ensure CppAd and boost compatibility
#include "physics/PinocchioRBD.h"

namespace ct {
namespace rbd {

template <class ROB, typename SCALAR = double>
class FixBaseRobotSystemLinear final : public ct::core::LinearSystem<2 * ROB::NJOINTS, ROB::NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2 * ROB::NJOINTS;
    static const size_t CONTROL_DIM = ROB::NJOINTS;

    using BASE = ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>;

    FixBaseRobotSystemLinear();

    FixBaseRobotSystemLinear(std::shared_ptr<PinocchioRBD<ROB, SCALAR>> rbd);

    FixBaseRobotSystemLinear(const FixBaseRobotSystemLinear& other);

    FixBaseRobotSystemLinear* clone() const override;

    //! compute the linear system A matrix (continuous time)
    const ct::core::StateMatrix<STATE_DIM, SCALAR>& getDerivativeState(
        const ct::core::StateVector<STATE_DIM, SCALAR>& x,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const SCALAR t = SCALAR(0.0)) override;

    //! compute the linear system B matrix (continuous time)
    const ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& getDerivativeControl(
        const ct::core::StateVector<STATE_DIM, SCALAR>& x,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const SCALAR t = SCALAR(0.0)) override;

    //! compute the linear system A and B matrices simultanteously (continuous time)
    void getDerivatives(ct::core::StateMatrix<STATE_DIM, SCALAR>& A,
        ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& B,
        const ct::core::StateVector<STATE_DIM, SCALAR>& x,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const SCALAR t = SCALAR(0.0)) override;

    void setRbd(std::shared_ptr<PinocchioRBD<ROB, SCALAR>> rbd);

    std::shared_ptr<PinocchioRBD<ROB, SCALAR>> getRbd();

protected:
    std::shared_ptr<PinocchioRBD<ROB, SCALAR>> rbd_;

    ct::core::StateMatrix<STATE_DIM, SCALAR> A_;
    ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> B_;
};

}  // namespace rbd
}  // namespace ct
