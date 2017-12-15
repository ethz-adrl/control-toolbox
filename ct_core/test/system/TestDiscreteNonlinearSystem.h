/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    TestDiscreteNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestDiscreteNonlinearSystem(SCALAR rate, std::shared_ptr<DiscreteController<2, 1, SCALAR>> controller = nullptr)
        : DiscreteControlledSystem<2, 1, SCALAR>(controller, SYSTEM_TYPE::GENERAL), rate_(rate)
    {
    }

    //copy constructor
    TestDiscreteNonlinearSystem(const TestDiscreteNonlinearSystem& arg)
        : DiscreteControlledSystem<2, 1, SCALAR>(arg), rate_(arg.rate_)
    {
    }


    virtual ~TestDiscreteNonlinearSystem() {}
    TestDiscreteNonlinearSystem* clone() const override { return new TestDiscreteNonlinearSystem(*this); }
    virtual void propagateControlledDynamics(const StateVector<STATE_DIM, SCALAR>& state,
        const int& n,
        const ControlVector<CONTROL_DIM, SCALAR>& control,
        StateVector<STATE_DIM, SCALAR>& stateNext) override
    {
        // this is pretty much random
        stateNext(0) = state(0) + rate_ * state(0) * control(0);
        stateNext(1) = state(0) * state(1) * state(1);
    }

private:
    SCALAR rate_;
};
}

typedef tpl::TestDiscreteNonlinearSystem<double> TestDiscreteNonlinearSystem;

}  // namespace core
}  // namespace ct
