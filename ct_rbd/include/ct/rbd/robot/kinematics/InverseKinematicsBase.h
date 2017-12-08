#pragma once

#include <ct/rbd/robot/kinematics/InverseKinematicsSolverBase.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class InverseKinematicsBase
{
public:
    virtual std::shared_ptr<InverseKinematicsSolverBase<NJOINTS, SCALAR>> get_solver(const int base,
        const int ee,
        const int id = 0) const = 0;
};

} /* namespace rbd */
} /* namespace ct */
