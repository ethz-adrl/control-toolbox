/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::DisturbanceObserver(
    std::shared_ptr<DisturbedSystem_t> system,
    const SensitivityApproximation_t& sensApprox,
    double dt,
    const ct::core::OutputStateMatrix<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& Caug,
    const ESTIMATOR& estimator,
    const ct::core::StateMatrix<ESTIMATE_DIM, SCALAR>& Qaug,
    const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R)
    : Base(system, sensApprox, dt, Caug, estimator, Qaug, R)
{
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::DisturbanceObserver(
    std::shared_ptr<DisturbedSystem_t> system,
    const SensitivityApproximation_t& sensApprox,
    const ESTIMATOR& estimator,
    const DisturbanceObserverSettings<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& do_settings)
    : Base(system, sensApprox, do_settings.dt, do_settings.C, estimator, do_settings.Qaug, do_settings.R)
{
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
typename DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::estimate_vector_t
DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::predict(const Time_t& t)
{
    return this->estimator_.template predict<CONTROL_DIM>(
        this->f_, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Zero(), this->Q_, t);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
typename DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::estimate_vector_t
DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::update(
    const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
    const Time_t&)
{
    return this->estimator_.template update<OUTPUT_DIM>(y, this->h_, this->R_);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
ct::core::StateVector<STATE_DIM, SCALAR>
DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::getStateEstimate()
{
    return this->estimator_.getEstimate().head(STATE_DIM);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
Eigen::Matrix<SCALAR, DIST_DIM, 1>
DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::getDisturbanceEstimate()
{
    return this->estimator_.getEstimate().tail(DIST_DIM);
}

}  // optcon
}  // ct
