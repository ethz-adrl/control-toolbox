/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>
#include <ct/core/types/EuclideanState.h>

namespace ct {
namespace core {

namespace tpl {

template <typename SCALAR, bool CONT_T>
class TestNonlinearSystem : public ControlledSystem<EuclideanState<2, SCALAR>, 1, CONT_T>
{
public:
    static const int STATE_DIM = 2;
    static const int CONTROL_DIM = 1;

    using Base = ControlledSystem<EuclideanState<2, SCALAR>, 1, CONT_T>;
    using Time_t = typename Base::Time_t;
    using Controller_t = typename Base::Controller_t;
    using control_vector_t = typename Base::control_vector_t;
    using state_vector_t = EuclideanState<2, SCALAR>;

    TestNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestNonlinearSystem(SCALAR w_n, std::shared_ptr<Controller_t> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), w_n_(w_n)
    {
    }

    TestNonlinearSystem(const TestNonlinearSystem& arg) : Base(arg), w_n_(arg.w_n_) {}
    virtual ~TestNonlinearSystem() {}
    TestNonlinearSystem* clone() const override { return new TestNonlinearSystem(*this); }
    void computeControlledDynamics(const state_vector_t& state,
        const Time_t& tn,
        const control_vector_t& control,
        typename state_vector_t::Tangent& dx) override
    {
        //this is pretty much random
        dx(0) = state(1) * state(0) + state(1) * control(0);
        dx(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
    }

private:
    SCALAR w_n_;
};
}  // namespace tpl

template <bool CONT_T>
using TestNonlinearSystem = tpl::TestNonlinearSystem<double, CONT_T>;

}  // namespace core
}  // namespace ct
