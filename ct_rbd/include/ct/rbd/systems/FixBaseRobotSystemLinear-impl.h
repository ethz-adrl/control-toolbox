#pragma once

namespace ct {
namespace rbd {

template <class ROB, typename SCALAR>
FixBaseRobotSystemLinear<ROB, SCALAR>::FixBaseRobotSystemLinear() : rbd_(nullptr)
{
}

template <class ROB, typename SCALAR>
FixBaseRobotSystemLinear<ROB, SCALAR>::FixBaseRobotSystemLinear(std::shared_ptr<PinocchioRBD<ROB, SCALAR>> rbd)
    : rbd_(rbd->clone())
{
}

template <class ROB, typename SCALAR>
FixBaseRobotSystemLinear<ROB, SCALAR>::FixBaseRobotSystemLinear(const FixBaseRobotSystemLinear& other)
    : BASE(other), rbd_(other.rbd_->clone())
{
}

template <class ROB, typename SCALAR>
auto FixBaseRobotSystemLinear<ROB, SCALAR>::clone() const -> FixBaseRobotSystemLinear*
{
    return new FixBaseRobotSystemLinear(*this);
}

template <class ROB, typename SCALAR>
auto FixBaseRobotSystemLinear<ROB, SCALAR>::getDerivativeState(const ct::core::StateVector<STATE_DIM, SCALAR>& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const SCALAR t) -> const ct::core::StateMatrix<STATE_DIM, SCALAR>&
{
    using DerivativeMatrix_t = Eigen::Matrix<SCALAR, ROB::NJOINTS, ROB::NJOINTS>;
    DerivativeMatrix_t partial_dq, partial_dv, partial_dtau;

    rbd_->computeForwardDynamicsDerivatives(
        x.template head<ROB::NJOINTS>(), x.template tail<ROB::NJOINTS>(), u, partial_dq, partial_dv, partial_dtau);

    A_.setZero();

    // the velocity mapping is represented by an identity block in the top right corner
    A_.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>().setIdentity();

    // the lower half of A consists of derivatives w.r.t. joint position and velocity
    A_.template bottomLeftCorner<STATE_DIM / 2, STATE_DIM / 2>() = partial_dq;
    A_.template bottomRightCorner<STATE_DIM / 2, STATE_DIM / 2>() = partial_dv;

    return A_;
}

template <class ROB, typename SCALAR>
auto FixBaseRobotSystemLinear<ROB, SCALAR>::getDerivativeControl(const ct::core::StateVector<STATE_DIM, SCALAR>& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const SCALAR t) -> const ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>&
{
    using DerivativeMatrix_t = Eigen::Matrix<SCALAR, ROB::NJOINTS, ROB::NJOINTS>;
    DerivativeMatrix_t partial_dq, partial_dv, partial_dtau;

    rbd_->computeForwardDynamicsDerivatives(
        x.template head<ROB::NJOINTS>(), x.template tail<ROB::NJOINTS>(), u, partial_dq, partial_dv, partial_dtau);

    B_.setZero();

    // the lower block of B simply consists of the inverse of the joint-space inertia matrix of the system
    B_.template bottomRows<CONTROL_DIM>() = partial_dtau;

    return B_;
}

template <class ROB, typename SCALAR>
void FixBaseRobotSystemLinear<ROB, SCALAR>::getDerivatives(ct::core::StateMatrix<STATE_DIM, SCALAR>& A,
    ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& B,
    const ct::core::StateVector<STATE_DIM, SCALAR>& x,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const SCALAR t)
{
    using DerivativeMatrix_t = Eigen::Matrix<SCALAR, ROB::NJOINTS, ROB::NJOINTS>;
    DerivativeMatrix_t partial_dq, partial_dv, partial_dtau;

    rbd_->computeForwardDynamicsDerivatives(
        x.template head<ROB::NJOINTS>(), x.template tail<ROB::NJOINTS>(), u, partial_dq, partial_dv, partial_dtau);


    // see explanation above
    A.setZero();
    A.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>().setIdentity();
    A.template bottomLeftCorner<STATE_DIM / 2, STATE_DIM / 2>() = partial_dq;
    A.template bottomRightCorner<STATE_DIM / 2, STATE_DIM / 2>() = partial_dv;

    // see explanation above
    B.setZero();
    B.template bottomRows<CONTROL_DIM>() = partial_dtau;
}

template <class ROB, typename SCALAR>
void FixBaseRobotSystemLinear<ROB, SCALAR>::setRbd(std::shared_ptr<PinocchioRBD<ROB, SCALAR>> rbd)
{
    rbd_ = rbd;
}

template <class ROB, typename SCALAR>
auto FixBaseRobotSystemLinear<ROB, SCALAR>::getRbd() -> std::shared_ptr<PinocchioRBD<ROB, SCALAR>>
{
    return rbd_;
}

}  // namespace rbd
}  // namespace ct