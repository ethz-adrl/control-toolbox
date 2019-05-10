/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "MeasurementModelBase.h"

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Linear Measurement Model is an interface for linear measurement models most commonly used in practice.
 *
 * @tparam STATE_DIM    nominal state dimensionality
 * @tparam DIST_DIM     dimensionality of the disturbance
 * @tparam CONTROL_DIM
 */
template <size_t OUTPUT_DIM, size_t STATE_DIM, typename SCALAR = double>
class LinearMeasurementModel : public MeasurementModelBase<OUTPUT_DIM, STATE_DIM, SCALAR>
{
public:
    using Base = MeasurementModelBase<OUTPUT_DIM, STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;
    using typename Base::output_vector_t;
    using typename Base::Time_t;
    using output_matrix_t = ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>;
    using output_state_matrix_t = ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR>;

    //! Calculates the measurement from the current state.
    virtual output_vector_t computeMeasurement(const state_vector_t& state, const Time_t& t = 0) = 0;
    //! Computes the derivative of the output w.r.t. the state.
    virtual output_state_matrix_t computeDerivativeState(const state_vector_t& state, const Time_t& t) = 0;
    //! Computes the derivative of the output w.r.t. the noise.
    virtual output_matrix_t computeDerivativeNoise(const state_vector_t& state, const Time_t& t) = 0;
};

}  // optcon
}  // ct
