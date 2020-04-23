#pragma once

#include <ct/core/core.h>  // always include ct-first to ensure CppAd and boost compatibility
#include "physics/RigidBodyDynamics.h"

namespace ct {
namespace rbd {

template <class ROB, typename SCALAR = double>
class FixBaseRobotSystem final : public ct::core::ControlledSystem<2 * ROB::NJOINTS, ROB::NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_DIM = 2 * ROB::NJOINTS;
    static const size_t CONTROL_DIM = ROB::NJOINTS;

    using BASE = ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>;

    FixBaseRobotSystem();

    FixBaseRobotSystem(std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>> rbd);

    FixBaseRobotSystem(const FixBaseRobotSystem& other);

    FixBaseRobotSystem* clone() const override;

    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
        ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override;

    void setRbd(std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>> rbd);

    std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>> getRbd();

protected:
    std::shared_ptr<RigidBodyDynamics<ROB, SCALAR>> rbd_;
};

}  // namespace rbd
}  // namespace ct
