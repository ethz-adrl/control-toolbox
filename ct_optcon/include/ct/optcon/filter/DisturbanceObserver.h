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
    using Base                       = StateObserver<OUTPUT_DIM, ESTIMATE_DIM, CONTROL_DIM, ESTIMATOR, SCALAR>;
    using typename Base::Time_t;

    DisturbanceObserver(std::shared_ptr<DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<ESTIMATE_DIM, CONTROL_DIM, ESTIMATE_DIM / 2, ESTIMATE_DIM / 2, SCALAR>&
            sensApprox,
        double dt,
        const ct::core::OutputStateMatrix<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& Caug,
        const ESTIMATOR& estimator,
        const ct::core::StateMatrix<ESTIMATE_DIM, SCALAR>& Qaug,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R)
        : Base(system, sensApprox, dt, Caug, estimator, Qaug, R)
    {
    }

    DisturbanceObserver(std::shared_ptr<DisturbedSystem<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<ESTIMATE_DIM, CONTROL_DIM, ESTIMATE_DIM / 2, ESTIMATE_DIM / 2, SCALAR>&
            sensApprox,
        const ESTIMATOR& estimator,
        const DisturbanceObserverSettings<OUTPUT_DIM, ESTIMATE_DIM, SCALAR>& do_settings)
        : Base(system, sensApprox, do_settings.dt, do_settings.C, estimator, do_settings.Qaug, do_settings.R)
    {
    }

    virtual ~DisturbanceObserver() {}
    ct::core::StateVector<ESTIMATE_DIM, SCALAR> predict(const Time_t& t = 0) override
    {
        return this->estimator_.template predict<CONTROL_DIM>(
            this->f_, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Zero(), this->Q_, t);
    }
    ct::core::StateVector<ESTIMATE_DIM, SCALAR> update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
        const Time_t& = 0) override
    {
        return this->estimator_.template update<OUTPUT_DIM>(y, this->h_, this->R_);
    }

    ct::core::StateVector<STATE_DIM, SCALAR> getStateEstimate()
    {
        return this->estimator_.getEstimate().head(STATE_DIM);
    }
    Eigen::Matrix<SCALAR, DIST_DIM, 1> getDisturbanceEstimate()
    {
        return this->estimator_.getEstimate().tail(DIST_DIM);
    }
};

}  // optcon
}  // ct
