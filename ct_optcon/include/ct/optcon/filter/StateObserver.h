/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FilterBase.h"
#include "ExtendedKalmanFilter.h"
#include "LinearMeasurementModel.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class StateObserver : public FilterBase<OBS_DIM, STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateObserver(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>&
            sensApprox,
        double dt,
        Eigen::Matrix<double, OBS_DIM, STATE_DIM> C,
        const ExtendedKalmanFilter<STATE_DIM, OBS_DIM, SCALAR>& ekf,
        const Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>& Q,
        const Eigen::Matrix<SCALAR, OBS_DIM, OBS_DIM>& R)
        : f_(system, sensApprox, dt), h_(C), ekf_(ekf), Q_(Q), R_(R)
    {
    }

    void filter() override {}
    virtual const ct::core::StateVector<STATE_DIM, SCALAR>& predict(ct::core::Time t = 0)
    {
        return ekf_.template predict<CONTROL_DIM>(f_, Eigen::Matrix<SCALAR, CONTROL_DIM, 1>::Zero(), Q_, t);
    }

    virtual const ct::core::StateVector<STATE_DIM, SCALAR>& update(const Eigen::Matrix<SCALAR, OBS_DIM, 1>& y,
        ct::core::Time t = 0)
    {
        return ekf_.update(y, h_, R_);
    }

protected:
    ExtendedKalmanFilter<STATE_DIM, OBS_DIM, SCALAR> ekf_;
    CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR> f_;
    LinearMeasurementModel<OBS_DIM, STATE_DIM, SCALAR> h_;
    Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> Q_;
    Eigen::Matrix<SCALAR, OBS_DIM, OBS_DIM> R_;
};

}  // optcon
}  // ct
