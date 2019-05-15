/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "LinearMeasurementModel.h"

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Linear Time-Invariant measurement model is simply a linear measurement model for which the matrix C is
 *        constant in time.
 *
 * @tparam STATE_DIM    nominal state dimensionality
 * @tparam DIST_DIM     dimensionality of the disturbance
 * @tparam CONTROL_DIM
 */
template <size_t OUTPUT_DIM, size_t STATE_DIM, typename SCALAR = double>
class LTIMeasurementModel : public LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>;
    using typename Base::output_matrix_t;
    using typename Base::output_state_matrix_t;
    using typename Base::output_vector_t;
    using typename Base::state_vector_t;
    using typename Base::Time_t;

    //! Default constructor.
    LTIMeasurementModel();

    //! Constructor.
    LTIMeasurementModel(const output_state_matrix_t& C, const output_matrix_t& dHdw = output_matrix_t::Zero());

    //! Calculates the measurement from the current state.
    output_vector_t computeMeasurement(const state_vector_t& state, const Time_t& t = 0) override;

    //! Returns matrix C.
    output_state_matrix_t computeDerivativeState(const state_vector_t& state, const Time_t& t) override;

    //! Returns the constant dHdw.
    output_matrix_t computeDerivativeNoise(const state_vector_t& state, const Time_t& t) override;

protected:
    output_state_matrix_t dHdx_;  //! Matrix C.
    output_matrix_t dHdw_;        //! Derivative of output w.r.t. noise.
};

}  // namespace optcon
}  // namespace ct
