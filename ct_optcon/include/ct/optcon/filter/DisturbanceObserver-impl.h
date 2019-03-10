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
    const output_estimate_matrix_t& Caug,
    const ESTIMATOR& estimator,
    const estimate_matrix_t& Qaug,
    const output_matrix_t& R)
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
auto DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::predict(
    const control_vector_t& u,
    const Time_t& t) -> estimate_vector_t
{
    return this->estimator_.template predict<CONTROL_DIM>(this->f_, u, this->Q_, t);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
auto DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::update(
    const output_vector_t& y,
    const Time_t&) -> estimate_vector_t
{
    return this->estimator_.template update<OUTPUT_DIM>(y, this->h_, this->R_);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
auto DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::getStateEstimate()
    -> state_vector_t
{
    return this->estimator_.getEstimate().template head<STATE_DIM>();
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
auto DisturbanceObserver<OUTPUT_DIM, STATE_DIM, DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::getDisturbanceEstimate()
    -> disturbance_vector_t
{
    return this->estimator_.getEstimate().template tail<DIST_DIM>();
}

}  // namespace optcon
}  // namespace ct
