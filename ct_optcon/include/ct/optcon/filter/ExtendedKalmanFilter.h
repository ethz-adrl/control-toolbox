/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "SystemModelBase.h"
#include "MeasurementModelBase.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t OBS_DIM, typename SCALAR = double>
class ExtendedKalmanFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ExtendedKalmanFilter(
        const ct::core::StateVector<STATE_DIM, SCALAR>& x0 = ct::core::StateVector<STATE_DIM, SCALAR>::Zero(),
        const Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>& P0 = Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>::Identity())
        : x_(x0), P_(P0)
    {
    }

    void setX(const ct::core::StateVector<STATE_DIM, SCALAR>& x) { x_ = x; }
    ExtendedKalmanFilter(const ExtendedKalmanFilter& arg) : x_(arg.x_), P_(arg.P_) {}

    template <size_t CONTROL_DIM>
    const ct::core::StateVector<STATE_DIM, SCALAR>& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>& Q,
        ct::core::Time t = 0)
    {
        f.updateJacobians(x_, u, t);
        x_ = f.computeDynamics(x_, u, t);
        P_ = (f.dFdx() * P_ * f.dFdx().transpose()) + f.dFdv() * Q * f.dFdv().transpose();
        return x_;
    }

    const ct::core::StateVector<STATE_DIM, SCALAR>& update(const Eigen::Matrix<SCALAR, OBS_DIM, 1>& y,
        MeasurementModelBase<OBS_DIM, STATE_DIM, SCALAR>& h,
        const Eigen::Matrix<SCALAR, OBS_DIM, OBS_DIM>& R)
    {
        h.updateJacobians(x_);
        const Eigen::Matrix<SCALAR, STATE_DIM, OBS_DIM> K =
            P_ * h.dHdx().transpose() *
            (h.dHdx() * P_ * h.dHdx().transpose() + h.dHdw() * R * h.dHdw().transpose()).inverse();

        x_ += K * (y - h.computeMeasurement(x_));
        P_ -= (K * h.dHdx() * P_).eval();

        return x_;
    }

    const ct::core::StateVector<STATE_DIM, SCALAR>& getEstimate() const { return x_; }
private:
    ct::core::StateVector<STATE_DIM, SCALAR> x_;
    Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> P_;
};

}  // optcon
}  // ct
