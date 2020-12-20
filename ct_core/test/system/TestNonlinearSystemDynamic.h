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

class TestNonlinearSystemDynamic : public ControlledSystem<EuclideanStateXd, Dynamic, ContinuousTime>
{
public:
    using state_vector_t = EuclideanStateXd;
    using Base = ControlledSystem<state_vector_t, Dynamic, ContinuousTime>;
    using Time_t = typename Base::Time_t;
    using Controller_t = typename Base::Controller_t;
    using control_vector_t = typename Base::control_vector_t;

    TestNonlinearSystemDynamic() = delete;

    TestNonlinearSystemDynamic(int state_dim, int control_dim, double w_n)
        : Base(SYSTEM_TYPE::GENERAL), state_dim_(state_dim), control_dim_(control_dim), w_n_(w_n)
    {
        ControlVectorXd u(control_dim_);
        u.setConstant(1.0);

        this->setController(std::shared_ptr<ConstantController<state_vector_t, Dynamic, ContinuousTime>>(
            new ConstantController<state_vector_t, Dynamic, ContinuousTime>(u)));
    }

    TestNonlinearSystemDynamic(const TestNonlinearSystemDynamic& arg)
        : Base(arg), state_dim_(arg.state_dim_), control_dim_(arg.control_dim_), w_n_(arg.w_n_)
    {
    }
    
    TestNonlinearSystemDynamic* clone() const override { return new TestNonlinearSystemDynamic(*this); }

    void computeControlledDynamics(const state_vector_t& state,
        const Time_t& tn,
        const control_vector_t& control,
        typename state_vector_t::Tangent& dx) override;

private:
    int state_dim_;
    int control_dim_;
    double w_n_;
};

}  // namespace core
}  // namespace ct
