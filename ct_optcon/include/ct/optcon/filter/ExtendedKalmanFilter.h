/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EstimatorBase.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR>
struct ExtendedKalmanFilterSettings;

/*!
 * \ingroup Filter
 *
 * \brief Extended Kalman Filter implementation. 
 * For an algorithmic overview, see also https://en.wikipedia.org/wiki/Extended_Kalman_filter
 */
template <size_t STATE_DIM, typename SCALAR = double>
class ExtendedKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    using Base = EstimatorBase<STATE_DIM, SCALAR>;
    using typename Base::state_matrix_t;
    using typename Base::state_vector_t;

    //! Constructor.
    ExtendedKalmanFilter(const state_vector_t& x0 = state_vector_t::Zero(),
        const state_matrix_t& P0 = state_matrix_t::Zero());

    //! Constructor from settings.
    ExtendedKalmanFilter(const ExtendedKalmanFilterSettings<STATE_DIM, SCALAR>& ekf_settings);

    //! Estimator predict method.
    template <size_t CONTROL_DIM>
    const state_vector_t& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const state_matrix_t& Q,
        const ct::core::Time& dt,
        const ct::core::Time& t);

    //! Estimator update method.
    template <size_t OUTPUT_DIM>
    const state_vector_t& update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
        const ct::core::Time& dt,
        const ct::core::Time& t);

    // return current covariance matrix
    const state_matrix_t& getCovarianceMatrix();

private:
    //! Covariance estimate
    state_matrix_t P_;
};

}  // namespace optcon
}  // namespace ct
