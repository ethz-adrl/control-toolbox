/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::StateObserver(
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
    const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>& sensApprox,
    const output_state_matrix_t& C,
    const output_matrix_t& D,
    const ESTIMATOR& estimator,
    const state_matrix_t& Q,
    const output_matrix_t& R,
    const state_matrix_t& dFdv)
    : estimator_(estimator), f_(system, sensApprox, dFdv), h_(C, D), Q_(Q), R_(R)
{
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::StateObserver(
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
    const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>& sensApprox,
    const ESTIMATOR& estimator,
    const StateObserverSettings<OUTPUT_DIM, STATE_DIM, SCALAR>& so_settings)
    : f_(system, sensApprox, so_settings.dFdv),
      h_(so_settings.C),
      estimator_(estimator),
      Q_(so_settings.Q),
      R_(so_settings.R)
{
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
auto StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::filter(const control_vector_t& u,
    const output_vector_t& y,
    const ct::core::Time& dt,
    const Time_t& t) -> state_vector_t
{
    predict(u, dt, t);
    return update(y, dt, t);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
auto StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::predict(const control_vector_t& u,
    const ct::core::Time& dt,
    const Time_t& t) -> state_vector_t
{
    return estimator_.template predict<CONTROL_DIM>(f_, u, Q_, dt, t);
}

template <size_t OUTPUT_DIM, size_t STATE_DIM, size_t CONTROL_DIM, class ESTIMATOR, typename SCALAR>
auto StateObserver<OUTPUT_DIM, STATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>::update(const output_vector_t& y,
    const ct::core::Time& dt,
    const Time_t& t) -> state_vector_t
{
    return estimator_.template update<OUTPUT_DIM>(y, h_, R_, dt, t);
}

}  // namespace optcon
}  // namespace ct
