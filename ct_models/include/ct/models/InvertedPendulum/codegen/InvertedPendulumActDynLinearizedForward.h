/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace models {
namespace InvertedPendulum {

class InvertedPendulumActDynLinearizedForward : public ct::core::LinearSystem<3, 1, double>
{
public:
    typedef ct::core::LinearSystem<3, 1, double> Base;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::state_matrix_t state_matrix_t;
    typedef typename Base::state_control_matrix_t state_control_matrix_t;

    InvertedPendulumActDynLinearizedForward(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<3, 1>(type)
    {
        initialize();
    }

    InvertedPendulumActDynLinearizedForward(const InvertedPendulumActDynLinearizedForward& other) { initialize(); }
    virtual ~InvertedPendulumActDynLinearizedForward(){};

    virtual InvertedPendulumActDynLinearizedForward* clone() const override { return new InvertedPendulumActDynLinearizedForward; }
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const double t = double(0.0)) override;

    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const double t = double(0.0)) override;

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
    std::array<double, 0> vX_;
    std::array<double, 0> vU_;
};
}
}
}
