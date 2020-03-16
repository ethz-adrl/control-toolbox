/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>

namespace ct {
namespace core {

namespace tpl {

template <typename SCALAR>
class TestDiscreteNonlinearSystem : public DiscreteControlledSystem<2, 1, SCALAR>
{
public:
    typedef DiscreteControlledSystem<2, 1, SCALAR> Base;
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;

    TestDiscreteNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestDiscreteNonlinearSystem(SCALAR rate, std::shared_ptr<DiscreteController<2, 1, SCALAR>> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), rate_(rate)
    {
    }

    //copy constructor
    TestDiscreteNonlinearSystem(const TestDiscreteNonlinearSystem& arg) : Base(arg), rate_(arg.rate_) {}
    virtual ~TestDiscreteNonlinearSystem() {}
    TestDiscreteNonlinearSystem* clone() const override { return new TestDiscreteNonlinearSystem(*this); }
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        // this is pretty much random
        stateNext(0) = state(0) + rate_ * state(0) * control(0);
        stateNext(1) = state(0) * state(1) * state(1);
    }

private:
    SCALAR rate_;
};
}  // namespace tpl

typedef tpl::TestDiscreteNonlinearSystem<double> TestDiscreteNonlinearSystem;

}  // namespace core
}  // namespace ct
