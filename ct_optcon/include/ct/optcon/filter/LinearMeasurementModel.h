/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "MeasurementModelBase.h"

namespace ct {
namespace optcon {

template <size_t OUTPUT_DIM, size_t STATE_DIM, typename SCALAR = double>
class LinearMeasurementModel : public MeasurementModelBase<OUTPUT_DIM, STATE_DIM, SCALAR>
{
public:
    using Base = MeasurementModelBase<OUTPUT_DIM, STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;
    using typename Base::output_vector_t;
    using typename Base::Time_t;
    using output_matrix_t       = ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>;
    using output_state_matrix_t = ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR>;

    virtual output_vector_t computeMeasurement(const state_vector_t& state, const Time_t& t = 0)       = 0;
    virtual output_state_matrix_t computeDerivativeState(const state_vector_t& state, const Time_t& t) = 0;
    virtual output_matrix_t computeDerivativeNoise(const state_vector_t& state, const Time_t& t)       = 0;
};

}  // optcon
}  // ct
