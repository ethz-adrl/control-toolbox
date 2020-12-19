/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include "TestNonlinearSystemDynamic.h"

namespace ct {
namespace core {

void TestNonlinearSystemDynamic::computeControlledDynamics(const state_vector_t& state,
    const Time_t& tn,
    const control_vector_t& control,
    typename state_vector_t::Tangent& dx)
{
    if (state.rows() != state_dim_)
        throw std::runtime_error("Dimension mismatch for state.");
    if (control.rows() != control_dim_)
        throw std::runtime_error("Dimension mismatch for control.");

    dx.resize(state_dim_);  // todo make it so that this is not necessary

    //this is pretty much random
    dx(0) = state(1) * state(0) + state(1) * control(0);
    dx(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
}

}  // namespace core
}  // namespace ct
