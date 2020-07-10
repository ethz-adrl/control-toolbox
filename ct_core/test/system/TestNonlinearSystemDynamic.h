/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>
#include <memory>
#include <iostream>
#include <ct/core/core.h>

namespace ct {
namespace core {

class TestNonlinearSystemDynamic : public ControlledSystem<EuclideanStateXd, -1, true>
{
public:
    // static const int STATE_DIM = 2;
    // static const int CONTROL_DIM = 1;

    using Base = ControlledSystem<EuclideanStateXd, -1, true>;
    using Time_t = typename Base::Time_t;
    using Controller_t = typename Base::Controller_t;
    using control_vector_t = typename Base::control_vector_t;
    using state_vector_t = EuclideanStateXd;

    TestNonlinearSystemDynamic() = delete;

    // constructor directly using frequency and damping coefficients
    TestNonlinearSystemDynamic(double w_n, std::shared_ptr<Controller_t> controller = nullptr)
        : Base(controller, SYSTEM_TYPE::GENERAL), w_n_(w_n)
    {
    }

    TestNonlinearSystemDynamic(const TestNonlinearSystemDynamic& arg) : Base(arg), w_n_(arg.w_n_) {}
    virtual ~TestNonlinearSystemDynamic() {}
    TestNonlinearSystemDynamic* clone() const override { return new TestNonlinearSystemDynamic(*this); }
    void computeControlledDynamics(const state_vector_t& state,
        const Time_t& tn,
        const control_vector_t& control,
        typename state_vector_t::Tangent& dx) override;

private:
    double w_n_;
};

}  // namespace core
}  // namespace ct
