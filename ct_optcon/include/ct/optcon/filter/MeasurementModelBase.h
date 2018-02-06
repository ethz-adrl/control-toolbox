/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, typename SCALAR = double>
class MeasurementModelBase
{
public:
    using state_vector_t  = ct::core::StateVector<STATE_DIM, SCALAR>;
    using output_vector_t = ct::core::OutputVector<OBS_DIM, SCALAR>;
    using Time_t          = ct::core::Time;

    virtual ~MeasurementModelBase() {}
    virtual ct::core::OutputVector<OBS_DIM, SCALAR> computeMeasurement(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::Time& t = 0) = 0;
};

}  // optcon
}  // ct
