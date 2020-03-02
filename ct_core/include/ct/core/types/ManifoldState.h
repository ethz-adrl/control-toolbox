/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Dense>
#include <manif/manif.h>

namespace ct {
namespace core {

template <typename MANIF_T, typename TAN>
class ManifoldState : public MANIF_T
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t TangentDim = TAN::DoF;

    using Scalar = typename MANIF_T::Scalar;
    using Tangent = TAN;
    using Base = MANIF_T;

    ManifoldState();
    virtual ~ManifoldState();

    template <typename OTHER>
    ManifoldState(const OTHER& other);
};

// todo do not move to implementation file
template <typename M, typename T>
template <typename OTHER>
ManifoldState<M, T>::ManifoldState(const OTHER& other) : Base(other)
{
}

} /* namespace core */
} /* namespace ct */
