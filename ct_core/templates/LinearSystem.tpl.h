/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace NS1 {
namespace NS2 {

class LINEAR_SYSTEM_NAME : public ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::state_matrix_t state_matrix_t;
    typedef typename Base::state_control_matrix_t state_control_matrix_t;

    LINEAR_SYSTEM_NAME(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>(type)
    {
        initialize();
    }

    LINEAR_SYSTEM_NAME(const LINEAR_SYSTEM_NAME& other) { initialize(); }
    virtual ~LINEAR_SYSTEM_NAME(){};

    virtual LINEAR_SYSTEM_NAME* clone() const override { return new LINEAR_SYSTEM_NAME; }
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t = SCALAR(0.0)) override;

    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t = SCALAR(0.0)) override;

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
    std::array<SCALAR, MAX_COUNT_STATE> vX_;
    std::array<SCALAR, MAX_COUNT_CONTROL> vU_;
};

}  // namespace NS2
}  // namespace NS1
}  // namespace ct
