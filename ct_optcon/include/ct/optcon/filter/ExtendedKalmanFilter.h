/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR = double>
class ExtendedKalmanFilter final : public EstimatorBase<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = EstimatorBase<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>;
    using typename Base::control_vector_t;
    using typename Base::output_matrix_t;
    using typename Base::output_vector_t;
    using typename Base::state_matrix_t;
    using typename Base::state_vector_t;

    //! Constructor.
    ExtendedKalmanFilter(std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
        std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
        const state_matrix_t& Q,
        const output_matrix_t& R,
        const state_vector_t& x0 = state_vector_t::Zero(),
        const state_matrix_t& P0 = state_matrix_t::Zero());

    //! Constructor from settings.
    ExtendedKalmanFilter(std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
        std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
        const ExtendedKalmanFilterSettings<STATE_DIM, SCALAR>& ekf_settings);

    //! Estimator predict method.
    const state_vector_t& predict(const control_vector_t& u,
        const ct::core::Time& dt,
        const ct::core::Time& t) override;

    //! Estimator update method.
    const state_vector_t& update(const output_vector_t& y, const ct::core::Time& dt, const ct::core::Time& t) override;

    // return current covariance matrix
    const state_matrix_t& getCovarianceMatrix();

    //! update Q matrix
    void setQ(const state_matrix_t& Q) { Q_ = Q; }
    //! update R matrix
    void setR(const output_matrix_t& R) { R_ = R; }
protected:
    //! Filter Q matrix.
    state_matrix_t Q_;

    //! Filter R matrix.
    output_matrix_t R_;

    //! Covariance estimate
    state_matrix_t P_;
};

}  // namespace optcon
}  // namespace ct
