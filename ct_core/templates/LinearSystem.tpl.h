/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

//#include <ct/core/core.h>

namespace ct {
namespace NS1 {
namespace NS2 {

class LINEAR_SYSTEM_NAME : public ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>
{
public:
    typedef typename Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef typename Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> state_control_matrix_t;

    LINEAR_SYSTEM_NAME(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>(type)
    {
        initialize();
    }

    LINEAR_SYSTEM_NAME(const LINEAR_SYSTEM_NAME& other) { initialize(); }
    virtual ~LINEAR_SYSTEM_NAME(){};

    virtual LINEAR_SYSTEM_NAME* clone() const override { return new LINEAR_SYSTEM_NAME; }
    virtual const state_matrix_t& getDerivativeState(const ct::core::StateVector<STATE_DIM>& x,
        const ct::core::ControlVector<CONTROL_DIM>& u,
        const double t = 0.0) override;

    virtual const state_control_matrix_t& getDerivativeControl(const ct::core::StateVector<STATE_DIM>& x,
        const ct::core::ControlVector<CONTROL_DIM>& u,
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
    std::array<double, MAX_COUNT_STATE> vX_;
    std::array<double, MAX_COUNT_CONTROL> vU_;
};
}
}
}
