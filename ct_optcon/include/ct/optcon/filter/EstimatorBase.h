/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FilterSettings.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR = double>
class EstimatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR>;

    EstimatorBase(const ct::core::StateVector<STATE_DIM, SCALAR>& x0 = ct::core::StateVector<STATE_DIM, SCALAR>::Zero())
        : x_est_(x0)
    {
    }
    EstimatorBase(const EstimatorBase& arg) : x_est_(arg.x_est_) {}

    const ct::core::StateVector<STATE_DIM, SCALAR>& getEstimate() const { return x_est_; }
    void setEstimate(const ct::core::StateVector<STATE_DIM, SCALAR>& x) { x_est_ = x; }

protected:
    ct::core::StateVector<STATE_DIM, SCALAR> x_est_;
};

}  // optcon
}  // ct
