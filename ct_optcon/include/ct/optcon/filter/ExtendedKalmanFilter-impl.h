/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR>
struct ExtendedKalmanFilterSettings;

template <size_t STATE_DIM, typename SCALAR>
ExtendedKalmanFilter<STATE_DIM, SCALAR>::ExtendedKalmanFilter(
    const typename ExtendedKalmanFilter<STATE_DIM, SCALAR>::state_vector_t& x0,
    const ct::core::StateMatrix<STATE_DIM, SCALAR>& P0)
    : Base(x0), P_(P0)
{
}

template <size_t STATE_DIM, typename SCALAR>
ExtendedKalmanFilter<STATE_DIM, SCALAR>::ExtendedKalmanFilter(
    const ExtendedKalmanFilterSettings<STATE_DIM, SCALAR>& ekf_settings)
    : Base(ekf_settings.x0), P_(ekf_settings.P0)
{
}

template <size_t STATE_DIM, typename SCALAR>
template <size_t CONTROL_DIM>
const typename ExtendedKalmanFilter<STATE_DIM, SCALAR>::state_vector_t&
ExtendedKalmanFilter<STATE_DIM, SCALAR>::predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
    const ct::core::Time& t)
{
    ct::core::StateMatrix<STATE_DIM, SCALAR> dFdx = f.computeDerivativeState(this->x_est_, u, t);
    ct::core::StateMatrix<STATE_DIM, SCALAR> dFdv = f.computeDerivativeNoise(this->x_est_, u, t);
    this->x_est_ = f.computeDynamics(this->x_est_, u, t);
    P_ = (dFdx * P_ * dFdx.transpose()) + dFdv * Q * dFdv.transpose();
    return this->x_est_;
}

template <size_t STATE_DIM, typename SCALAR>
template <size_t OUTPUT_DIM>
const typename ExtendedKalmanFilter<STATE_DIM, SCALAR>::state_vector_t& ExtendedKalmanFilter<STATE_DIM, SCALAR>::update(
    const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
    LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
    const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
    const ct::core::Time& t)
{
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR> dHdx = h.computeDerivativeState(this->x_est_, t);
    ct::core::OutputMatrix<OUTPUT_DIM, SCALAR> dHdw = h.computeDerivativeNoise(this->x_est_, t);
    const Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM> K =
        P_ * dHdx.transpose() * (dHdx * P_ * dHdx.transpose() + dHdw * R * dHdw.transpose()).inverse();

    this->x_est_ += K * (y - h.computeMeasurement(this->x_est_));
    P_ -= (K * dHdx * P_).eval();

    return this->x_est_;
}

}  // optcon
}  // ct
