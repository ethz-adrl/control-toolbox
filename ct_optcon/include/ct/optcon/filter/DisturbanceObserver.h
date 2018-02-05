/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "StateObserver.h"
#include "DisturbedSystem.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DisturbanceObserver : public StateObserver<OBS_DIM, STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = StateObserver<OBS_DIM, STATE_DIM + DIST_DIM, CONTROL_DIM, SCALAR>;

    DisturbanceObserver() {}
    DisturbanceObserver(std::shared_ptr<DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<STATE_DIM + DIST_DIM,
                            CONTROL_DIM,
                            (STATE_DIM + DIST_DIM) / 2,
                            (STATE_DIM + DIST_DIM) / 2,
                            SCALAR>& sensApprox,
        double dt,
        const Eigen::Matrix<double, OBS_DIM, STATE_DIM + DIST_DIM>& Caug,
        const ExtendedKalmanFilter<STATE_DIM + DIST_DIM, OBS_DIM, SCALAR>& ekf,
        const Eigen::Matrix<SCALAR, STATE_DIM + DIST_DIM, STATE_DIM + DIST_DIM>& Qaug,
        const Eigen::Matrix<SCALAR, OBS_DIM, OBS_DIM>& R)
        : Base(system, sensApprox, dt, Caug, ekf, Qaug, R)
    {
    }

    void filter() override {}
    const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& predict(ct::core::Time t = 0) override
    {
        return this->ekf_.template predict<CONTROL_DIM>(
            this->f_, Eigen::Matrix<SCALAR, CONTROL_DIM, 1>::Zero(), this->Q_, t);
    }

    const ct::core::StateVector<STATE_DIM + DIST_DIM, SCALAR>& update(const Eigen::Matrix<SCALAR, OBS_DIM, 1>& y,
        ct::core::Time t = 0) override
    {
        return this->ekf_.update(y, this->h_, this->R_);
    }

    Eigen::Matrix<SCALAR, STATE_DIM, 1> getStateEstimate() { return this->ekf_.getEstimate().head(STATE_DIM); }
    Eigen::Matrix<SCALAR, DIST_DIM, 1> getDisturbanceEstimate() { return this->ekf_.getEstimate().tail(DIST_DIM); }
};

}  // optcon
}  // ct
