/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include "TestSystemDynamic.h"

namespace ct {
namespace core {


TestSystemDynamic::TestSystemDynamic(int state_dim, int control_dim, double w_n)
    : Base(SYSTEM_TYPE::GENERAL), state_dim_(state_dim), control_dim_(control_dim), w_n_(w_n)
{
    if (control_dim_ > state_dim_)
        throw std::runtime_error("TestSystemDynamic: control-dim must not be greater than state-dim.");
}

void TestSystemDynamic::computeControlledDynamics(const EuclideanStateXd& state,
    const Time_t& tn,
    const control_vector_t& control,
    typename EuclideanStateXd::Tangent& dx)
{
    if (state.rows() != state_dim_)
        throw std::runtime_error("Dimension mismatch for state.");
    if (control.rows() != control_dim_)
        throw std::runtime_error("Dimension mismatch for control.");

    dx.resize(state_dim_);

    //this is pretty much random
    dx(0) = state(1) * state(0) + state(1) * control(0);
    dx(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
}

TestSystemDynamic::TestSystemDynamic(const TestSystemDynamic& arg)
    : Base(arg), state_dim_(arg.state_dim_), control_dim_(arg.control_dim_), w_n_(arg.w_n_)
{
}

TestSystemDynamic* TestSystemDynamic::clone() const
{
    return new TestSystemDynamic(*this);
}

}  // namespace core
}  // namespace ct
