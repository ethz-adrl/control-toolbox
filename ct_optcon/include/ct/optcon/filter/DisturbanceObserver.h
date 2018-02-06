/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "StateObserver.h"
#include "DisturbedSystem.h"
#include "ExtendedKalmanFilter.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM,
    size_t STATE_DIM,
    size_t DIST_DIM,
    size_t CONTROL_DIM,
    class ESTIMATOR,
    typename SCALAR = double>
class DisturbanceObserver : public StateObserver<OBS_DIM, STATE_DIM + DIST_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t ESTIMATE_DIM = STATE_DIM + DIST_DIM;

    DisturbanceObserver() {}
    DisturbanceObserver(std::shared_ptr<DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<ESTIMATE_DIM, CONTROL_DIM, ESTIMATE_DIM / 2, ESTIMATE_DIM / 2, SCALAR>&
            sensApprox,
        double dt,
        const Eigen::Matrix<double, OBS_DIM, ESTIMATE_DIM>& Caug,
        const ESTIMATOR& ekf,
        const Eigen::Matrix<SCALAR, ESTIMATE_DIM, ESTIMATE_DIM>& Qaug,
        const Eigen::Matrix<SCALAR, OBS_DIM, OBS_DIM>& R)
        : StateObserver<OBS_DIM, ESTIMATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>(system,
              sensApprox,
              dt,
              Caug,
              ekf,
              Qaug,
              R)
    {
    }

    const ct::core::StateVector<ESTIMATE_DIM, SCALAR>& predict(const ct::core::Time& t = 0) override
    {
        return this->estimator_.template predict<CONTROL_DIM>(
            this->f_, Eigen::Matrix<SCALAR, CONTROL_DIM, 1>::Zero(), this->Q_, t);
    }
    const ct::core::StateVector<ESTIMATE_DIM, SCALAR>& update(const ct::core::OutputVector<OBS_DIM, SCALAR>& y,
        const ct::core::Time& = 0) override
    {
        return this->estimator_.template update<OBS_DIM>(y, this->h_, this->R_);
    }

    Eigen::Matrix<SCALAR, STATE_DIM, 1> getStateEstimate() {
        return this->estimator_.getEstimate().head(STATE_DIM);
    }
    Eigen::Matrix<SCALAR, DIST_DIM, 1> getDisturbanceEstimate()
    {
        return this->estimator_.getEstimate().tail(DIST_DIM);
    }
};

}  // optcon
}  // ct
