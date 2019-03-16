/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR>
ExtendedKalmanFilter<STATE_DIM, SCALAR>::ExtendedKalmanFilter(
    const typename ExtendedKalmanFilter<STATE_DIM, SCALAR>::state_vector_t& x0,
    const state_matrix_t& P0)
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
auto ExtendedKalmanFilter<STATE_DIM, SCALAR>::predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const state_matrix_t& Q,
    const ct::core::Time& dt,
    const ct::core::Time& t) -> const state_vector_t&
{
    // STEP 1 - compute covariance matrix prediction

    // the system is linearized at the current control input, but using the state estimate from the previous timestep.
    state_matrix_t dFdx = f.computeDerivativeState(this->x_est_, u, dt, t);
    state_matrix_t dFdv = f.computeDerivativeNoise(this->x_est_, u, dt, t);

    // compute covariance update
    P_ = (dFdx * P_ * dFdx.transpose()) + dFdv * (dt * Q) * dFdv.transpose();

    // STEP 2 - compute state prediction (based on last state esimate but current control input)

    this->x_est_ = f.computeDynamics(this->x_est_, u, dt, t);

    return this->x_est_;
}

template <size_t STATE_DIM, typename SCALAR>
template <size_t OUTPUT_DIM>
auto ExtendedKalmanFilter<STATE_DIM, SCALAR>::update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
    LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
    const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
    const ct::core::Time& dt,
    const ct::core::Time& t) -> const state_vector_t&
{
    // STEP 1 - compute residual convariances
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR> dHdx = h.computeDerivativeState(this->x_est_, t);
    ct::core::OutputMatrix<OUTPUT_DIM, SCALAR> dHdw = h.computeDerivativeNoise(this->x_est_, t);

    // STEP 2 - compute near-optimal Kalman gain
    const Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM> K =
        P_ * dHdx.transpose() * (dHdx * P_ * dHdx.transpose() + dHdw * (R) * dHdw.transpose()).inverse();

    // STEP 3 - state estimate correction
    this->x_est_ += K * (y - h.computeMeasurement(this->x_est_));

    // STEP 4 - covariance matrix correction
    P_ -= (K * dHdx * P_).eval();

    return this->x_est_;
}

template <size_t STATE_DIM, typename SCALAR>
auto ExtendedKalmanFilter<STATE_DIM, SCALAR>::getCovarianceMatrix() -> const state_matrix_t&
{
    return P_;
}

}  // namespace optcon
}  // namespace ct
