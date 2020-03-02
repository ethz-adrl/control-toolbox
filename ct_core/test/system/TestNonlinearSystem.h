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

template <typename SCALAR>
class TestNonlinearSystem : public ControlledSystem<EuclideanState<2, SCALAR>, 1, SCALAR>
{
public:
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef ControlVector<1, SCALAR> control_vector_t;
    typedef EuclideanState<2, SCALAR> state_vector_t;

    TestNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestNonlinearSystem(SCALAR w_n, std::shared_ptr<Controller<state_vector_t, 1, SCALAR>> controller = nullptr)
        : ControlledSystem<state_vector_t, 1, SCALAR>(controller, SYSTEM_TYPE::GENERAL), w_n_(w_n)
    {
    }

    TestNonlinearSystem(const TestNonlinearSystem& arg)
        : ControlledSystem<state_vector_t, 1, SCALAR>(arg), w_n_(arg.w_n_)
    {
    }

    virtual ~TestNonlinearSystem() {}

    TestNonlinearSystem* clone() const override { return new TestNonlinearSystem(*this); }

    virtual void computeControlledDynamics(const StateVector<2, SCALAR>& state,
        const SCALAR& t,
        const control_vector_t& control,
        typename state_vector_t::Tangent& derivative) override
    {
        //this is pretty much random
        derivative(0) = state(1) * state(0) + state(1) * control(0);
        derivative(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
    }

private:
    SCALAR w_n_;
};
}  // namespace tpl

typedef tpl::TestNonlinearSystem<double> TestNonlinearSystem;

}  // namespace core
}  // namespace ct
