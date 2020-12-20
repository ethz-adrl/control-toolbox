/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace core {

class TestSystemDynamic : public ControlledSystem<EuclideanStateXd, ContinuousTime>
{
public:
    using Base = ControlledSystem<EuclideanStateXd, ContinuousTime>;
    using Time_t = typename Base::Time_t;
    using Controller_t = typename Base::Controller_t;
    using control_vector_t = typename Base::control_vector_t;

    TestSystemDynamic() = delete;

    TestSystemDynamic(int state_dim, int control_dim, double w_n);

    TestSystemDynamic(const TestSystemDynamic& arg);

    TestSystemDynamic* clone() const override;

    void computeControlledDynamics(const EuclideanStateXd& state,
        const Time_t& tn,
        const control_vector_t& control,
        typename EuclideanStateXd::Tangent& dx) override;

private:
    int state_dim_;
    int control_dim_;
    double w_n_;
};

}  // namespace core
}  // namespace ct
