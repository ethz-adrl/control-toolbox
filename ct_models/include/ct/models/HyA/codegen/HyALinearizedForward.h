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

namespace tpl {

template <typename SCALAR>
class HyALinearizedForward : public ct::core::LinearSystem<12, 6, SCALAR>
{
public:
    typedef typename Eigen::Matrix<SCALAR, 12, 12> state_matrix_t;
    typedef typename Eigen::Matrix<SCALAR, 12, 6> state_control_matrix_t;

    HyALinearizedForward(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<12, 6, SCALAR>(type)
    {
        initialize();
    }

    HyALinearizedForward(const HyALinearizedForward<SCALAR>& other) { initialize(); }
    virtual ~HyALinearizedForward(){};

    virtual HyALinearizedForward* clone() const override { return new HyALinearizedForward; }
    virtual const state_matrix_t& getDerivativeState(const ct::core::StateVector<12, SCALAR>& x,
        const ct::core::ControlVector<6, SCALAR>& u,
        const SCALAR t = SCALAR(0.0)) override;

    virtual const state_control_matrix_t& getDerivativeControl(const ct::core::StateVector<12, SCALAR>& x,
        const ct::core::ControlVector<6, SCALAR>& u,
        const SCALAR t = SCALAR(0.0)) override;

private:
    void initialize()
    {
        dFdx_.setZero();
        dFdu_.setZero();
        vX_.fill(SCALAR(0.0));
        vU_.fill(SCALAR(0.0));
    }

    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;
    std::array<SCALAR, 392> vX_;
    std::array<SCALAR, 69> vU_;
};
}

typedef tpl::HyALinearizedForward<double> HyALinearizedForward;
}
}
}
