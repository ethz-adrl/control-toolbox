#pragma once

#include "DiscreteTrajectoryBase.h"
#include <manif/manif.h>

namespace ct {
namespace core {

template <typename MANIF_TYPE, typename SCALAR = double>
class ManifTrajectory final : public DiscreteTrajectoryBase<ManifTrajectory<MANIF_TYPE, SCALAR>, MANIF_TYPE>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = DiscreteTrajectoryBase<ManifTrajectory<SCALAR>, MANIF_TYPE>;

    ManifTrajectory() : BASE() {}

    virtual MANIF_TYPE eval(const SCALAR& evalTime) override
    {
        // a linear interpolation scheme for SE3 - spherical interpolation
        auto manifInterpFun = [](const auto& p0, const auto& p1, const auto& alpha) {
            return manif::interpolate(p0, p1, 1.0 - alpha);
        };

        MANIF_TYPE result;
        this->interp_.interpolate(this->time_, this->data_, evalTime, result, manifInterpFun);

        return result;
    }
};

namespace tpl {

template <SCALAR>
using SE3Trajectory = ManifTrajectory<manif::SE3<SCALAR>>;

template <SCALAR>
using SE2Trajectory = ManifTrajectory<manif::SE2<SCALAR>>;

}  // namespace tpl

using SE3Trajectory = tpl::SE3Trajectory<double>;
using SE2Trajectory = tpl::SE2Trajectory<double>;

}  // namespace core
}  // namespace ct