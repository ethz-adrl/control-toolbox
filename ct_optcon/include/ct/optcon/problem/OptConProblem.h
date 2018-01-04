/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

//
// template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR=double>
// class OptConProblem : public ct::optcon::OptConProblemBase<STATE_DIM,
//     CONTROL_DIM,
//     core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
//     core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
//     SCALAR>
// {
//   typedef OptConProblemBase<STATE_DIM,
//       CONTROL_DIM,
//       core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
//       core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
//       SCALAR> Base;
//   using Base::Base; // inherit all constructors
// };


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using OptConProblem = OptConProblemBase<STATE_DIM,
    CONTROL_DIM,
    core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
    core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>,
    core::SystemLinearizer<STATE_DIM, CONTROL_DIM, SCALAR>,
    SCALAR>;


}  // namespace optcon
}  // namespace ct
