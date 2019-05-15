/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
SteadyStateKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::SteadyStateKalmanFilter(
    std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
    std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
    const state_matrix_t& Q,
    const output_matrix_t& R,
    const state_vector_t& x0,
    size_t maxDAREIterations)
    : Base(f, h, x0), Q_(Q), R_(R), maxDAREIterations_(maxDAREIterations)
{
    P_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
SteadyStateKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::SteadyStateKalmanFilter(
    std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
    std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
    const SteadyStateKalmanFilterSettings<STATE_DIM, SCALAR>& sskf_settings)
    : Base(f, h, sskf_settings.x0),
      Q_(sskf_settings.Q),
      R_(sskf_settings.R),
      maxDAREIterations_(sskf_settings.maxDAREIterations)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
auto SteadyStateKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::predict(const control_vector_t& u,
    const ct::core::Time& dt,
    const ct::core::Time& t) -> const state_vector_t&
{
    A_ = this->f_->computeDerivativeState(this->x_est_, u, t);
    this->x_est_ = this->f_->computeDynamics(this->x_est_, u, t);
    return this->x_est_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
auto SteadyStateKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::update(const output_vector_t& y,
    const ct::core::Time& dt,
    const ct::core::Time& t) -> const state_vector_t&
{
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR> dHdx = this->h_->computeDerivativeState(this->x_est_, t);
    Eigen::Matrix<SCALAR, OUTPUT_DIM, STATE_DIM> K;

    DARE<STATE_DIM, OUTPUT_DIM, SCALAR> dare;
    try
    {
        P_ = dare.computeSteadyStateRiccatiMatrix(
            Q_, R_, A_.transpose(), dHdx.transpose(), P_, K, false, 1e-6, maxDAREIterations_);
    } catch (...)
    {
        throw;
    }

    this->x_est_ -= K.transpose() * (y - this->h_->computeMeasurement(this->x_est_, t));
    return this->x_est_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR>
void SteadyStateKalmanFilter<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>::setMaxDAREIterations(size_t maxDAREIterations)
{
    maxDAREIterations_ = maxDAREIterations;
}

}  // namespace optcon
}  // namespace ct
