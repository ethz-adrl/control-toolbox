/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EstimatorBase.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR>
struct ExtendedKalmanFilterSettings;

template <size_t STATE_DIM, typename SCALAR = double>
class ExtendedKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    using Base                  = EstimatorBase<STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;

    ExtendedKalmanFilter(const state_vector_t& x0 = state_vector_t::Zero(),
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& P0 = ct::core::StateMatrix<STATE_DIM, SCALAR>::Zero())
        : Base(x0), P_(P0)
    {
    }

    ExtendedKalmanFilter(const ExtendedKalmanFilterSettings<STATE_DIM, SCALAR>& ekf_settings)
        : Base(ekf_settings.x0), P_(ekf_settings.P0)
    {
    }

    template <size_t CONTROL_DIM>
    const state_vector_t& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
        const ct::core::Time& t = 0)
    {
        ct::core::StateMatrix<STATE_DIM, SCALAR> dFdx = f.computeDerivativeState(this->x_est_, u, t);
        ct::core::StateMatrix<STATE_DIM, SCALAR> dFdv = f.computeDerivativeNoise(this->x_est_, u, t);
        this->x_est_ = f.computeDynamics(this->x_est_, u, t);
        P_           = (dFdx * P_ * dFdx.transpose()) + dFdv * Q * dFdv.transpose();
        return this->x_est_;
    }

    template <size_t OBS_DIM>
    const state_vector_t& update(const ct::core::OutputVector<OBS_DIM, SCALAR>& y,
        LinearMeasurementModel<OBS_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OBS_DIM, SCALAR>& R,
        const ct::core::Time& t = 0)
    {
        ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, SCALAR> dHdx = h.computeDerivativeState(this->x_est_, t);
        ct::core::OutputMatrix<OBS_DIM, SCALAR> dHdw = h.computeDerivativeNoise(this->x_est_, t);
        const Eigen::Matrix<SCALAR, STATE_DIM, OBS_DIM> K =
            P_ * dHdx.transpose() * (dHdx * P_ * dHdx.transpose() + dHdw * R * dHdw.transpose()).inverse();

        this->x_est_ += K * (y - h.computeMeasurement(this->x_est_));
        P_ -= (K * dHdx * P_).eval();

        return this->x_est_;
    }

private:
    ct::core::StateMatrix<STATE_DIM, SCALAR> P_;
};

}  // optcon
}  // ct
