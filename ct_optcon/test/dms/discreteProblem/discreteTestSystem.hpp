/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>

namespace ct {
namespace optcon {

namespace tpl {

template <typename SCALAR>
class TestDiscreteNonlinearSystem : public core::DiscreteControlledSystem<2, 1, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef core::DiscreteControlledSystem<2, 1, SCALAR> Base;
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;

    TestDiscreteNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestDiscreteNonlinearSystem(SCALAR rate, std::shared_ptr<core::DiscreteController<2, 1, SCALAR>> controller = nullptr)
        : Base(controller, core::SYSTEM_TYPE::GENERAL), rate_(rate)
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
        stateNext(1) = SCALAR(0.1) * state(0) * state(1) * state(1);
    }

private:
    SCALAR rate_;
};
}  // namespace tpl

typedef tpl::TestDiscreteNonlinearSystem<double> TestDiscreteNonlinearSystem;

}  // namespace optcon
}  // namespace ct
