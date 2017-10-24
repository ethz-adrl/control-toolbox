/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>


namespace ct {
namespace models {
namespace HyQ {

class HyQBareModelLinearizedReverse : public ct::core::LinearSystem<36, 12>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename Eigen::Matrix<double, 36, 36> state_matrix_t;
    typedef typename Eigen::Matrix<double, 36, 12> state_control_matrix_t;

    HyQBareModelLinearizedReverse(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<36, 12>(type)
    {
        initialize();
    }

    HyQBareModelLinearizedReverse(const HyQBareModelLinearizedReverse& other) { initialize(); }
    virtual ~HyQBareModelLinearizedReverse(){};

    virtual HyQBareModelLinearizedReverse* clone() const override { return new HyQBareModelLinearizedReverse; }
    virtual const state_matrix_t& getDerivativeState(const ct::core::StateVector<36>& x,
        const ct::core::ControlVector<12>& u,
        const double t = 0.0) override;

    virtual const state_control_matrix_t& getDerivativeControl(const ct::core::StateVector<36>& x,
        const ct::core::ControlVector<12>& u,
        const double t = 0.0) override;

private:
    void initialize()
    {
        dFdx_.setZero();
        dFdu_.setZero();
    }

    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;
    std::array<double, 3788> vX_;
    std::array<double, 383> vU_;
};
}
}
}
