#pragma once

namespace ct {
namespace rbd {

template <class ROB, typename SCALAR>
FixBaseRobotSystem<ROB, SCALAR>::FixBaseRobotSystem() : rbd_(nullptr)
{
}

template <class ROB, typename SCALAR>
FixBaseRobotSystem<ROB, SCALAR>::FixBaseRobotSystem(std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>> rbd)
    : rbd_(rbd->clone())
{
}

template <class ROB, typename SCALAR>
FixBaseRobotSystem<ROB, SCALAR>::FixBaseRobotSystem(const FixBaseRobotSystem& other)
    : BASE(other), rbd_(other.rbd_->clone())
{
}

template <class ROB, typename SCALAR>
auto FixBaseRobotSystem<ROB, SCALAR>::clone() const -> FixBaseRobotSystem*
{
    return new FixBaseRobotSystem(*this);
}

template <class ROB, typename SCALAR>
void FixBaseRobotSystem<ROB, SCALAR>::computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& t,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
    ct::core::StateVector<STATE_DIM, SCALAR>& derivative)
{
    // map joint velocities (block-mapping) to derivative vector.
    derivative.template head<ROB::NJOINTS>() = state.template tail<ROB::NJOINTS>();

    // compute accelerations using forward dynamics
    Eigen::Matrix<SCALAR, ROB::NJOINTS, 1> accOut;
    rbd_->computeForwardDynamics(
        state.template head<ROB::NJOINTS>(), state.template tail<ROB::NJOINTS>(), controlIn.toImplementation(), accOut);

    // map joint accelerations to derivative vector
    derivative.template tail<ROB::NJOINTS>() = accOut;
}

template <class ROB, typename SCALAR>
void FixBaseRobotSystem<ROB, SCALAR>::setRbd(std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>> rbd)
{
    rbd_ = rbd;
}

template <class ROB, typename SCALAR>
auto FixBaseRobotSystem<ROB, SCALAR>::getRbd() -> std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>>
{
    return rbd_;
}

}  // namespace rbd
}  // namespace ct
