/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR = double>
class EstimatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EstimatorBase(const ct::core::StateVector<STATE_DIM, SCALAR>& x0 = ct::core::StateVector<STATE_DIM, SCALAR>::Zero())
        : x_(x0)
    {
    }
    EstimatorBase(const EstimatorBase& arg) : x_(arg.x_) {}

    const ct::core::StateVector<STATE_DIM, SCALAR>& getEstimate() const { return x_; }
    void setEstimate(const ct::core::StateVector<STATE_DIM, SCALAR>& x) { x_ = x; }

protected:
    ct::core::StateVector<STATE_DIM, SCALAR> x_;
};

}  // optcon
}  // ct
