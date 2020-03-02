#pragma once

#include "DiscreteTrajectoryBase.h"
#include <Eigen/Dense>

namespace ct {
namespace core {

/*!
 * @brief implementation of a discrete trajectory for eigen-based quantities living in classical Euclidean spaces
 * @note the documentations of the constructors are given in the base class
 * 
 * @todo solve this in a clean way using traits
 */
template <class T, class ALLOC = Eigen::aligned_allocator<T>, typename SCALAR = double>
class EuclideanTrajectory final : public DiscreteTrajectoryBase<EuclideanTrajectory<T, ALLOC, SCALAR>, T, ALLOC, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Scalar_t = SCALAR;
    using EuclideanTrajectory_t = EuclideanTrajectory<T, ALLOC, SCALAR>;
    using BASE = DiscreteTrajectoryBase<EuclideanTrajectory_t, T, ALLOC, SCALAR>;

    EuclideanTrajectory(const InterpolationType& type = ZOH) : BASE(type) {}

    EuclideanTrajectory(const tpl::TimeArray<Scalar_t>& time,
        const DiscreteArray<T, ALLOC>& data,
        const InterpolationType& type = ZOH)
        : BASE(time, data, type)
    {
    }

    EuclideanTrajectory(const DiscreteArray<T, ALLOC>& data,
        const Scalar_t& deltaT,
        const Scalar_t& t0,
        const InterpolationType& type = ZOH)
        : BASE(data, deltaT, t0, type)
    {
    }

    EuclideanTrajectory(const EuclideanTrajectory_t& other) : BASE(other) {}

    EuclideanTrajectory(EuclideanTrajectory_t& other, const size_t startIndex, const size_t endIndex)
        : BASE(other, startIndex, endIndex)
    {
    }

    // override base class function at compile time
    T eval_specialized(const Scalar_t& evalTime)
    {
        T result;
        this->interp_.interpolate(this->time_, this->data_, evalTime, result);
        return result;
    }
};

}  // namespace core
}  // namespace ct