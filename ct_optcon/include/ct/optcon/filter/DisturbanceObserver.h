/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "StateObserver.h"
#include "DisturbedSystem.h"
#include "DisturbedSystemController.h"

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Disturbance observer works in the same way as the state observer, except that it assumes a state disturbance.
 *        The disturbance is included by augmenting the state (appending to the end) of the disturbance estimates.
 *
 * @tparam OUTPUT_DIM   dimensionality of the output
 * @tparam STATE_DIM    nominal state dimensionality
 * @tparam DIST_DIM     dimensionality of the disturbance
 * @tparam CONTROL_DIM  dimensionality of the control input
 * @tparam ESTIMATOR    underlying estimator class
 */
template <size_t OUTPUT_DIM,
    size_t STATE_DIM,
    size_t DIST_DIM,
    size_t CONTROL_DIM,
    class ESTIMATOR,
    typename SCALAR = double>
class DisturbanceObserver : public StateObserver<OUTPUT_DIM, STATE_DIM + DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t ESTIMATE_DIM = STATE_DIM + DIST_DIM;
    using Base = StateObserver<OUTPUT_DIM, ESTIMATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>;

    using DisturbedSystem_t = DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>;

    using SensitivityApproximation_t =
        ct::core::SensitivityApproximation<ESTIMATE_DIM, CONTROL_DIM, ESTIMATE_DIM / 2, ESTIMATE_DIM / 2, SCALAR>;

    using typename Base::control_vector_t;
    using typename Base::output_matrix_t;
    using typename Base::output_vector_t;
    using typename Base::Time_t;

    using estimate_vector_t = ct::core::StateVector<ESTIMATE_DIM, SCALAR>;
    using estimate_matrix_t = ct::core::StateMatrix<ESTIMATE_DIM, SCALAR>;
    using state_vector_t = ct::core::StateVector<STATE_DIM, SCALAR>;
    using disturbance_vector_t = Eigen::Matrix<SCALAR, DIST_DIM, 1>;
    using output_estimate_matrix_t = ct::core::OutputStateMatrix<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>;

    //! Constructor. We assume a linear model, i.e. C matrix is constant.
    DisturbanceObserver(std::shared_ptr<DisturbedSystem_t> system,
        const SensitivityApproximation_t& sensApprox,
        const output_estimate_matrix_t& Caug,
        const ESTIMATOR& estimator,
        const estimate_matrix_t& Qaug,
        const output_matrix_t& R,
        const estimate_matrix_t& dFdv);

    //! Constructor from observer settings.
    DisturbanceObserver(std::shared_ptr<DisturbedSystem_t> system,
        const SensitivityApproximation_t& sensApprox,
        const ESTIMATOR& estimator,
        const DisturbanceObserverSettings<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& do_settings);

    //! Destructor.
    virtual ~DisturbanceObserver() = default;

    //! Prediction step of the estimation.
    estimate_vector_t predict(const control_vector_t& u, const ct::core::Time& dt, const Time_t& t) override;

    //! Update step of the estimation.
    estimate_vector_t update(const output_vector_t& y, const ct::core::Time& dt, const Time_t& t) override;

    //! State estimate getter.
    state_vector_t getStateEstimate();

    //! Disturbance estimate getter.
    disturbance_vector_t getDisturbanceEstimate();

    //! get covariance matrix
    const estimate_matrix_t& getCovarianceMatrix();
};

}  // namespace optcon
}  // namespace ct
