/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../lqr/riccati/DARE.hpp"
#include "EstimatorBase.h"
#include "LinearMeasurementModel.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR>
struct SteadyStateKalmanFilterSettings;

/*!
 * \ingroup Filter
 *
 * \brief Steady State Kalman Filter is a time invariant linear estimator. It starts with the same update rule as the
 *        standard Kalman Filter, but instead of propagating the covariance and estimate through time, it assumes
 *        convergence reducing the problem to solving an Algebraic Ricatti Equation.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR = double>
class SteadyStateKalmanFilter final : public EstimatorBase<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = EstimatorBase<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>;
    using typename Base::control_vector_t;
    using typename Base::output_matrix_t;
    using typename Base::output_vector_t;
    using typename Base::state_matrix_t;
    using typename Base::state_vector_t;

    //! Constructor.
    SteadyStateKalmanFilter(std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
        std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
        const state_matrix_t& Q,
        const output_matrix_t& R,
        const state_vector_t& x0 = state_vector_t::Zero(),
        size_t maxDAREIterations = 1000);

    //! Constructor from settings.
    SteadyStateKalmanFilter(std::shared_ptr<SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>> f,
        std::shared_ptr<LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>> h,
        const SteadyStateKalmanFilterSettings<STATE_DIM, SCALAR>& sskf_settings);

    //! Estimator predict method.
    const state_vector_t& predict(const control_vector_t& u,
        const ct::core::Time& dt,
        const ct::core::Time& t) override;

    //! Estimator update method.
    const state_vector_t& update(const output_vector_t& y, const ct::core::Time& dt, const ct::core::Time& t) override;

    //! Limit number of iterations of the DARE solver.
    void setMaxDAREIterations(size_t maxDAREIterations);

private:
    size_t maxDAREIterations_;
    state_matrix_t P_;  //! Covariance estimate.
    state_matrix_t A_;  //! Computed linearized system matrix
    output_matrix_t R_;
    state_matrix_t Q_;
};

}  // namespace optcon
}  // namespace ct
