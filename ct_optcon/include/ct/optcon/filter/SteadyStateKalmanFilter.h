/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EstimatorBase.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM_T, typename SCALAR = double>
class SteadyStateKalmanFilter : public EstimatorBase<STATE_DIM_T, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SteadyStateKalmanFilter(
        const ct::core::StateVector<STATE_DIM_T, SCALAR>& x0 = ct::core::StateVector<STATE_DIM_T, SCALAR>::Zero(),
        size_t maxDAREIterations = 1000)
        : EstimatorBase<STATE_DIM_T, SCALAR>(x0), maxDAREIterations_(maxDAREIterations)
    {
        P_.setIdentity();
    }

    template <size_t CONTROL_DIM>
    const ct::core::StateVector<STATE_DIM_T, SCALAR>& predict(SystemModelBase<STATE_DIM_T, CONTROL_DIM, SCALAR>& f,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const Eigen::Matrix<SCALAR, STATE_DIM_T, STATE_DIM_T>& Q,
        ct::core::Time t = 0)
    {
        f.updateJacobians(this->x_, u, t);
        this->x_ = f.computeDynamics(this->x_, u, t);
        Q_       = Q;
        A_       = f.dFdx();
        return this->x_;
    }

    template <size_t OBS_DIM>
    const ct::core::StateVector<STATE_DIM_T, SCALAR>& update(const Eigen::Matrix<SCALAR, OBS_DIM, 1>& y,
        MeasurementModelBase<OBS_DIM, STATE_DIM_T, SCALAR>& h,
        const Eigen::Matrix<SCALAR, OBS_DIM, OBS_DIM>& R)
    {
        h.updateJacobians(this->x_);
        Eigen::Matrix<SCALAR, OBS_DIM, STATE_DIM_T> K;

        DARE<STATE_DIM_T, OBS_DIM, SCALAR> dare;
        try
        {
            P_ = dare.computeSteadyStateRiccatiMatrix(
                Q_, R, A_.transpose(), h.dHdx().transpose(), P_, K, false, 1e-6, maxDAREIterations_);
        } catch (...)
        {
            throw;
        }

        this->x_ += K.transpose() * (y - h.computeMeasurement(this->x_));
        return this->x_;
    }

    void setMaxDAREIterations(size_t maxDAREIterations) { maxDAREIterations_ = maxDAREIterations; }
private:
    size_t maxDAREIterations_;
    Eigen::Matrix<SCALAR, STATE_DIM_T, STATE_DIM_T> P_;
    Eigen::Matrix<SCALAR, STATE_DIM_T, STATE_DIM_T> A_;
    Eigen::Matrix<SCALAR, STATE_DIM_T, STATE_DIM_T> Q_;
};

}  // optcon
}  // ct
