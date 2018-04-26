/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
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
 * @tparam STATE_DIM
 */
template <size_t STATE_DIM, typename SCALAR = double>
class SteadyStateKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    using Base = EstimatorBase<STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;

    //! Constructor.
    SteadyStateKalmanFilter(const state_vector_t& x0 = state_vector_t::Zero(), size_t maxDAREIterations = 1000);

    //! Constructor from settings.
    SteadyStateKalmanFilter(const SteadyStateKalmanFilterSettings<STATE_DIM, SCALAR>& sskf_settings);

    //! Estimator predict method.
    template <size_t CONTROL_DIM>
    const state_vector_t& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
        const ct::core::Time& t = 0);

    //! Estimator update method.
    template <size_t OUTPUT_DIM>
    const state_vector_t& update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
        const ct::core::Time& t = 0);

    //! Limit number of iterations of the DARE solver.
    void setMaxDAREIterations(size_t maxDAREIterations);

private:
    size_t maxDAREIterations_;
    ct::core::StateMatrix<STATE_DIM, SCALAR> P_;  //! Covariance estimate.
    ct::core::StateMatrix<STATE_DIM, SCALAR> A_;  //! Computed linearized system matrix
    ct::core::StateMatrix<STATE_DIM, SCALAR> Q_;  //! System covariance matrix.
};

}  // optcon
}  // ct
