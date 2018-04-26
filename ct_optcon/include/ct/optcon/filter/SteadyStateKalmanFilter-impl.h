/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR>
struct SteadyStateKalmanFilterSettings;

template <size_t STATE_DIM, typename SCALAR>
SteadyStateKalmanFilter<STATE_DIM, SCALAR>::SteadyStateKalmanFilter(const state_vector_t& x0, size_t maxDAREIterations)
    : Base(x0), maxDAREIterations_(maxDAREIterations)
{
    P_.setZero();
}

template <size_t STATE_DIM, typename SCALAR>
SteadyStateKalmanFilter<STATE_DIM, SCALAR>::SteadyStateKalmanFilter(
    const SteadyStateKalmanFilterSettings<STATE_DIM, SCALAR>& sskf_settings)
    : Base(sskf_settings.x0), maxDAREIterations_(sskf_settings.maxDAREIterations)
{
}

template <size_t STATE_DIM, typename SCALAR>
template <size_t CONTROL_DIM>
const typename SteadyStateKalmanFilter<STATE_DIM, SCALAR>::state_vector_t&
SteadyStateKalmanFilter<STATE_DIM, SCALAR>::predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
    const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
    const ct::core::Time& t)
{
    A_ = f.computeDerivativeState(this->x_est_, u, t);
    Q_ = Q;
    this->x_est_ = f.computeDynamics(this->x_est_, u, t);
    return this->x_est_;
}

template <size_t STATE_DIM, typename SCALAR>
template <size_t OUTPUT_DIM>
const typename SteadyStateKalmanFilter<STATE_DIM, SCALAR>::state_vector_t&
SteadyStateKalmanFilter<STATE_DIM, SCALAR>::update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
    LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
    const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
    const ct::core::Time& t)
{
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR> dHdx = h.computeDerivativeState(this->x_est_, t);
    Eigen::Matrix<SCALAR, OUTPUT_DIM, STATE_DIM> K;

    DARE<STATE_DIM, OUTPUT_DIM, SCALAR> dare;
    try
    {
        P_ = dare.computeSteadyStateRiccatiMatrix(
            Q_, R, A_.transpose(), dHdx.transpose(), P_, K, false, 1e-6, maxDAREIterations_);
    } catch (...)
    {
        throw;
    }

    this->x_est_ -= K.transpose() * (y - h.computeMeasurement(this->x_est_, t));
    return this->x_est_;
}

template <size_t STATE_DIM, typename SCALAR>
void SteadyStateKalmanFilter<STATE_DIM, SCALAR>::setMaxDAREIterations(size_t maxDAREIterations)
{
    maxDAREIterations_ = maxDAREIterations;
}

}  // optcon
}  // ct
