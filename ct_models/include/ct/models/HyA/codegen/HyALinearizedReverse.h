/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace models {
namespace HyA {

class HyALinearizedReverse : public ct::core::LinearSystem<12, 6>
{
public:
    typedef typename Eigen::Matrix<double, 12, 12> state_matrix_t;
    typedef typename Eigen::Matrix<double, 12, 6> state_control_matrix_t;

    HyALinearizedReverse(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<12, 6>(type)
    {
        initialize();
    }

    HyALinearizedReverse(const HyALinearizedReverse& other) { initialize(); }
    virtual ~HyALinearizedReverse(){};

    virtual HyALinearizedReverse* clone() const override { return new HyALinearizedReverse; }
    virtual const state_matrix_t& getDerivativeState(const ct::core::StateVector<12>& x,
        const ct::core::ControlVector<6>& u,
        const double t = 0.0) override;

    virtual const state_control_matrix_t& getDerivativeControl(const ct::core::StateVector<12>& x,
        const ct::core::ControlVector<6>& u,
        const double t = 0.0) override;

private:
    void initialize()
    {
        dFdx_.setZero();
        dFdu_.setZero();
        vX_.fill(0.0);
        vU_.fill(0.0);
    }

    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;
    std::array<double, 975> vX_;
    std::array<double, 75> vU_;
};
}
}
}
