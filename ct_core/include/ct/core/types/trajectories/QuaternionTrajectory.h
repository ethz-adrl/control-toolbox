#pragma once

#include "DiscreteTrajectoryBase.h"
#include <Eigen/Dense>

namespace ct {
namespace core {

/*!
 * @brief implementation of a discrete trajectory for quaternion-types requiring SLERP as interpolation scheme
 * @note the documentations of the constructors are given in the base class
 */
template <typename SCALAR = double, class T = Eigen::Quaternion<SCALAR>, class ALLOC = Eigen::aligned_allocator<T>>
class QuaternionTrajectory final
    : public DiscreteTrajectoryBase<QuaternionTrajectory<SCALAR, T, ALLOC>, T, ALLOC, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Scalar_t = SCALAR;
    using Quaternion_t = T;
    using QuaternionTrajectory_t = QuaternionTrajectory<T, ALLOC, SCALAR>;
    using BASE = DiscreteTrajectoryBase<QuaternionTrajectory<T, ALLOC, SCALAR>, T, ALLOC, SCALAR>;

    QuaternionTrajectory(const InterpolationType& type = ZOH) : BASE(type) {}

    QuaternionTrajectory(const tpl::TimeArray<Scalar_t>& time,
        const DiscreteArray<T, ALLOC>& data,
        const InterpolationType& type = ZOH)
        : BASE(time, data, type)
    {
    }

    QuaternionTrajectory(const DiscreteArray<T, ALLOC>& data,
        const Scalar_t& deltaT,
        const Scalar_t& t0,
        const InterpolationType& type = ZOH)
        : BASE(data, deltaT, t0, type)
    {
    }

    QuaternionTrajectory(const QuaternionTrajectory_t& other) : BASE(other) {}

    QuaternionTrajectory(QuaternionTrajectory_t& other, const size_t startIndex, const size_t endIndex)
        : BASE(other, startIndex, endIndex)
    {
    }

    Quaternion_t eval_specialized(const Scalar_t& evalTime)
    {
        // custom interpolation lambda functional for quaternion types
        auto interp_functional = [](const auto q1, const auto q2, const auto alpha) { return q1.slerp(alpha, q2); };

        Quaternion_t result;
        this->interp_.interpolate(this->time_, this->data_, evalTime, result, interp_functional);

        return result;
    }
};

}  // namespace core
}  // namespace ct