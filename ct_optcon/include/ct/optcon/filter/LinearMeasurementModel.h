/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "MeasurementModelBase.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, typename SCALAR = double>
class LinearMeasurementModel : public MeasurementModelBase<OBS_DIM, STATE_DIM, SCALAR>
{
public:
    virtual ~LinearMeasurementModel() {}

    virtual ct::core::OutputVector<OBS_DIM, SCALAR> computeMeasurement(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t = 0) = 0;
    virtual ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, SCALAR> computeDerivativeState(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t) = 0;
    virtual ct::core::OutputMatrix<OBS_DIM, SCALAR> computeDerivativeNoise(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t) = 0;
};

}  // optcon
}  // ct
