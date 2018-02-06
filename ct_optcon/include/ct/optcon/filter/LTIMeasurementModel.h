/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "LinearMeasurementModel.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, typename SCALAR = double>
class LTIMeasurementModel : public LinearMeasurementModel<OBS_DIM, STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LTIMeasurementModel()
    {
        dHdx_.setZero();
        dHdw_.setIdentity();
    }

    LTIMeasurementModel(const ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, SCALAR>& C,
        const ct::core::OutputMatrix<OBS_DIM, SCALAR>& dHdw = ct::core::OutputMatrix<OBS_DIM, SCALAR>::Identity())
        : dHdx_(C)
    {
    }
    ct::core::OutputVector<OBS_DIM, SCALAR> computeMeasurement(const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t = 0) override
    {
        return dHdx_ * state;
    }

    ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, SCALAR> computeDerivativeState(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t) override
    {
        return dHdx_;
    }
    ct::core::OutputMatrix<OBS_DIM, SCALAR> computeDerivativeNoise(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t) override
    {
        return dHdw_;
    }

protected:
    ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, SCALAR> dHdx_;
    ct::core::OutputMatrix<OBS_DIM, SCALAR> dHdw_;
};

}  // optcon
}  // ct
