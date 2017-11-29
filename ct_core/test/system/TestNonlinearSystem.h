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
class TestNonlinearSystem : public ControlledSystem<2, 1, SCALAR>
{
public:
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    TestNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestNonlinearSystem(SCALAR w_n, std::shared_ptr<Controller<2, 1, SCALAR>> controller = nullptr)
        : ControlledSystem<2, 1, SCALAR>(controller, SYSTEM_TYPE::GENERAL), w_n_(w_n)
    {
    }

    TestNonlinearSystem(const TestNonlinearSystem& arg) : ControlledSystem<2, 1, SCALAR>(arg), w_n_(arg.w_n_) {}
    virtual ~TestNonlinearSystem() {}
    TestNonlinearSystem* clone() const override { return new TestNonlinearSystem(*this); }
    virtual void computeControlledDynamics(const StateVector<2, SCALAR>& state,
        const SCALAR& t,
        const ControlVector<1, SCALAR>& control,
        StateVector<2, SCALAR>& derivative) override
    {
        //this is pretty much random
        derivative(0) = state(1) * state(0) + state(1) * control(0);
        derivative(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
    }

private:
    SCALAR w_n_;
};
}

typedef tpl::TestNonlinearSystem<double> TestNonlinearSystem;

}  // namespace core
}  // namespace ct
