/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "StateObserver.h"
#include "DisturbedSystem.h"
#include "DisturbedSystemController.h"
#include "ExtendedKalmanFilter.h"

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

    using typename Base::Time_t;
    using estimate_vector_t = ct::core::StateVector<ESTIMATE_DIM, SCALAR>;

    //! Constructor. We assume a linear model, i.e. C matrix is constant.
    DisturbanceObserver(std::shared_ptr<DisturbedSystem_t> system,
        const SensitivityApproximation_t& sensApprox,
        double dt,
        const ct::core::OutputStateMatrix<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& Caug,
        const ESTIMATOR& estimator,
        const ct::core::StateMatrix<ESTIMATE_DIM, SCALAR>& Qaug,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R);

    //! Constructor from observer settings.
    DisturbanceObserver(std::shared_ptr<DisturbedSystem_t> system,
        const SensitivityApproximation_t& sensApprox,
        const ESTIMATOR& estimator,
        const DisturbanceObserverSettings<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& do_settings);

    //! Destructor.
    virtual ~DisturbanceObserver() = default;

    //! Prediction step of the estimation.
    estimate_vector_t predict(const Time_t& t = 0) override;

    //! Update step of the estimation.
    estimate_vector_t update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y, const Time_t& = 0) override;

    //! State estimate getter.
    ct::core::StateVector<STATE_DIM, SCALAR> getStateEstimate();

    //! Disturbance estimate getter.
    Eigen::Matrix<SCALAR, DIST_DIM, 1> getDisturbanceEstimate();
};

}  // optcon
}  // ct
