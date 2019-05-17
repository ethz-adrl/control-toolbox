/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::ExtendedKalmanFilter(
    std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
    std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
    const state_matrix_t& Q,
    const output_matrix_t& R,
    const state_vector_t& x0,
    const state_matrix_t& P0)
    : Base(f, h, x0), Q_(Q), R_(R), P_(P0)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::ExtendedKalmanFilter(
    std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
    std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
    const ExtendedKalmanFilterSettings<STATE_DIM, SCALAR>& ekf_settings)
    : Base(f, h, ekf_settings.x0), Q_(ekf_settings.Q), R_(ekf_settings.R), P_(ekf_settings.P0)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
auto ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::predict(const control_vector_t& u,
    const ct::core::Time& dt,
    const ct::core::Time& t) -> const state_vector_t&
{
    // STEP 1 - compute covariance matrix prediction

    // the system is linearized at the current control input, but using the state estimate from the previous timestep.
    state_matrix_t dFdx = this->f_->computeDerivativeState(this->x_est_, u, dt, t);
    state_matrix_t dFdv = this->f_->computeDerivativeNoise(this->x_est_, u, dt, t);

    // compute covariance update
    P_ = (dFdx * P_ * dFdx.transpose()) + dFdv * (dt * Q_) * dFdv.transpose();

    // STEP 2 - compute state prediction (based on last state esimate but current control input)

    this->x_est_ = this->f_->computeDynamics(this->x_est_, u, dt, t);

    return this->x_est_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
auto ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::update(const output_vector_t& y,
    const ct::core::Time& dt,
    const ct::core::Time& t) -> const state_vector_t&
{
    // STEP 1 - compute residual convariances
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR> dHdx = this->h_->computeDerivativeState(this->x_est_, t);
    output_matrix_t dHdw = this->h_->computeDerivativeNoise(this->x_est_, t);

    // STEP 2 - compute near-optimal Kalman gain
    const Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM> K =
        P_ * dHdx.transpose() * (dHdx * P_ * dHdx.transpose() + dHdw * (R_)*dHdw.transpose()).inverse();

    // STEP 3 - state estimate correction
    this->x_est_ += K * (y - this->h_->computeMeasurement(this->x_est_));

    // STEP 4 - covariance matrix correction
    P_ -= (K * dHdx * P_).eval();

    return this->x_est_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
auto ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::getCovarianceMatrix() -> const state_matrix_t&
{
    return P_;
}

}  // namespace optcon
}  // namespace ct
