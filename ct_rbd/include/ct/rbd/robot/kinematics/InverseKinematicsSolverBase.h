#pragma once

#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class InverseKinematicsSolverBase
{
public:
    const std::vector<std::vector<SCALAR>>& computeInverseKinematics(tpl::RigidBodyPose<SCALAR> ee,
        const std::vector<SCALAR> free_joints) const = 0;
};

} /* namespace rbd */
} /* namespace ct */
