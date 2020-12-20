/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>
#include <ct/core/types/ManifoldState.h>

namespace ct {
namespace core {

class TestSystemSE2 : public ControlledSystem<ct::core::SE2d, ContinuousTime>
{
public:
    using Base = ControlledSystem<ct::core::SE2d, ContinuousTime>;
    using Time_t = typename Base::Time_t;
    using Controller_t = typename Base::Controller_t;
    using State = ct::core::SE2d;

    TestSystemSE2() = delete;

    TestSystemSE2(SCALAR d, std::shared_ptr<Controller_t> controller, SE2d m_ref = SE2d::Identity())
        : Base(controller, SYSTEM_TYPE::GENERAL), d_(d), m_ref_(m_ref)
    {
    }

    TestSystemSE2(const TestSystemSE2& arg) : Base(arg), d_(arg.d_) {}

    TestSystemSE2* clone() const override { return new TestSystemSE2(*this); }

    void computeControlledDynamics(const State& m,
        const Time_t& tn,
        const ControlVectord& control,
        typename State::Tangent& dx) override
    {
        auto xtan = m_ref_.rminus(m); // express error w.r.t. m
        dx(0) = d_ * xtan(0) + control(0);
        dx(1) = d_ * xtan(1) + control(1);
        dx(2) = d_ * xtan(2) + control(2);
    }

private:
    SCALAR d_;
    SE2d m_ref_;
};

}  // namespace core
}  // namespace ct