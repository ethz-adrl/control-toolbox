/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, typename SCALAR = double>
class MeasurementModelBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeasurementModelBase() {}
    virtual ~MeasurementModelBase() {}
    virtual void updateJacobians(const ct::core::StateVector<STATE_DIM, SCALAR>& state) = 0;
    virtual ct::core::StateVector<OBS_DIM, SCALAR> computeMeasurement(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state) = 0;

    virtual Eigen::Matrix<double, OBS_DIM, STATE_DIM>& dHdx() = 0;
    virtual Eigen::Matrix<double, OBS_DIM, OBS_DIM>& dHdw()   = 0;
};

}  // optcon
}  // ct
