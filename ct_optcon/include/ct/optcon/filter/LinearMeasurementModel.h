/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "MeasurementModelBase.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, typename SCALAR = double>
class LinearMeasurementModel : public MeasurementModelBase<OBS_DIM, STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinearMeasurementModel()
    {
        dHdx_.setZero();
        dHdw_.setIdentity();
    }

    LinearMeasurementModel(Eigen::Matrix<double, OBS_DIM, STATE_DIM> C,
        const Eigen::Matrix<double, OBS_DIM, OBS_DIM>& dHdw = Eigen::Matrix<double, OBS_DIM, OBS_DIM>::Identity())
        : dHdx_(C)
    {
        dHdw_.setIdentity();
    }
    ct::core::StateVector<OBS_DIM, SCALAR> computeMeasurement(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state) override
    {
        return dHdx_ * state;
    }

    void updateJacobians(const ct::core::StateVector<STATE_DIM, SCALAR>& state) override {}
    Eigen::Matrix<double, OBS_DIM, STATE_DIM>& dHdx() override { return dHdx_; }
    Eigen::Matrix<double, OBS_DIM, OBS_DIM>& dHdw() override { return dHdw_; }
protected:
    Eigen::Matrix<double, OBS_DIM, STATE_DIM> dHdx_;
    Eigen::Matrix<double, OBS_DIM, OBS_DIM> dHdw_;
};

}  // optcon
}  // ct
