#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace ct {
namespace core {

/*
 * \brief returns true if an eigen-type is nan
 */
template <typename Derived>
inline bool eigen_is_nan(const Eigen::MatrixBase<Derived>& x)
{
    return ((x.array() != x.array())).all();
}

}  // namespace core
}  // namespace ct