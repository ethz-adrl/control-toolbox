/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using ContinuousOptConProblem = OptConProblemBase<STATE_DIM,
    CONTROL_DIM,
    core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
    core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
    core::SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>,
    SCALAR>;


}  // namespace optcon
}  // namespace ct
