/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

//TODO if necessary, let DiscreteOptConProblem derive properly from
// OptConProblemBase and add constructor such that a discrete problem can be
// initialized from a continuous one

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using DiscreteOptConProblem = OptConProblemBase<STATE_DIM,
    CONTROL_DIM,
    core::DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
    core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
    core::DiscreteSystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>,
    SCALAR>;

}  // namespace optcon
}  // namespace ct
