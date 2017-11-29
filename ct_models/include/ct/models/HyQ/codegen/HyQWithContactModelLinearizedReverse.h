/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>


namespace ct {
namespace models {
namespace HyQ {

class HyQWithContactModelLinearizedReverse : public ct::core::LinearSystem<36, 12>
{
public:
    typedef typename Eigen::Matrix<double, 36, 36> state_matrix_t;
    typedef typename Eigen::Matrix<double, 36, 12> state_control_matrix_t;

    HyQWithContactModelLinearizedReverse(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<36, 12>(type)
    {
        initialize();
    }

    HyQWithContactModelLinearizedReverse(const HyQWithContactModelLinearizedReverse& other) { initialize(); }
    virtual ~HyQWithContactModelLinearizedReverse(){};

    virtual HyQWithContactModelLinearizedReverse* clone() const override
    {
        return new HyQWithContactModelLinearizedReverse;
    }

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
        vX_.fill(0.0);
        vU_.fill(0.0);
    }

    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;
    std::array<double, 9379> vX_;
    std::array<double, 383> vU_;
};
}
}
}
