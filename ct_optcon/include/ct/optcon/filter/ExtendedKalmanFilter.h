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
 * \brief Extended Kalman Filter is a nonlinear estimator. It works by applying the same estimate rules as the standard
 *        Kalman Filter, but it does it on a linearization around the current state and covariance estimates.
 *
 * @tparam STATE_DIM
 */
template <size_t STATE_DIM, typename SCALAR = double>
class ExtendedKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    using Base = EstimatorBase<STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;

    //! Constructor.
    ExtendedKalmanFilter(const state_vector_t& x0 = state_vector_t::Zero(),
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& P0 = ct::core::StateMatrix<STATE_DIM, SCALAR>::Zero());

    //! Constructor from settings.
    ExtendedKalmanFilter(const ExtendedKalmanFilterSettings<STATE_DIM, SCALAR>& ekf_settings);

    //! Estimator predict method.
    template <size_t CONTROL_DIM>
    const state_vector_t& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
        const ct::core::Time& t = 0);

    //! Estimator update method.
    template <size_t OUTPUT_DIM>
    const state_vector_t& update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
        const ct::core::Time& t = 0);

private:
    ct::core::StateMatrix<STATE_DIM, SCALAR> P_;  //! Covariance estimate.
};

}  // optcon
}  // ct
