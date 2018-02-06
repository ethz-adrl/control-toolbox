/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EstimatorBase.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, typename SCALAR = double>
class SteadyStateKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;

    SteadyStateKalmanFilter(
        const ct::core::StateVector<STATE_DIM, SCALAR>& x0 = ct::core::StateVector<STATE_DIM, SCALAR>::Zero(),
        size_t maxDAREIterations = 1000)
        : EstimatorBase<STATE_DIM, SCALAR>(x0), maxDAREIterations_(maxDAREIterations)
    {
        P_.setZero();
    }

    template <size_t CONTROL_DIM>
    const ct::core::StateVector<STATE_DIM, SCALAR>& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
        const ct::core::Time& t = 0)
    {
        A_           = f.computeDerivativeState(this->x_est_, u, t);
        Q_           = Q;
        this->x_est_ = f.computeDynamics(this->x_est_, u, t);
        return this->x_est_;
    }

    template <size_t OBS_DIM>
    const ct::core::StateVector<STATE_DIM, SCALAR>& update(const ct::core::OutputVector<OBS_DIM, SCALAR>& y,
        LinearMeasurementModel<OBS_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OBS_DIM, SCALAR>& R,
        const ct::core::Time& t = 0)
    {
        ct::core::OutputStateMatrix<OBS_DIM, STATE_DIM, SCALAR> dHdx = h.computeDerivativeState(this->x_est_, t);
        Eigen::Matrix<SCALAR, OBS_DIM, STATE_DIM> K;

        DARE<STATE_DIM, OBS_DIM, SCALAR> dare;
        try
        {
            P_ = dare.computeSteadyStateRiccatiMatrix(
                Q_, R, A_.transpose(), dHdx.transpose(), P_, K, false, 1e-6, maxDAREIterations_);
        } catch (...)
        {
            throw;
        }

        this->x_est_ += K.transpose() * (y - h.computeMeasurement(this->x_est_, t));
        return this->x_est_;
    }

    void setMaxDAREIterations(size_t maxDAREIterations) { maxDAREIterations_ = maxDAREIterations; }
private:
    size_t maxDAREIterations_;
    ct::core::StateMatrix<STATE_DIM, SCALAR> P_;
    ct::core::StateMatrix<STATE_DIM, SCALAR> A_;
    ct::core::StateMatrix<STATE_DIM, SCALAR> Q_;
};

}  // optcon
}  // ct
