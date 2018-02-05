/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SystemModelBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SystemModelBase() {}
    virtual ~SystemModelBase() {}
    virtual void updateJacobians(const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::Time t) = 0;
    virtual ct::core::StateVector<STATE_DIM, SCALAR> computeDynamics(
        const ct::core::StateVector<STATE_DIM, SCALAR>& state,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::Time t) = 0;

    virtual Eigen::Matrix<double, STATE_DIM, STATE_DIM>& dFdx() = 0;
    virtual Eigen::Matrix<double, STATE_DIM, STATE_DIM>& dFdv() = 0;
};

}  // optcon
}  // ct
