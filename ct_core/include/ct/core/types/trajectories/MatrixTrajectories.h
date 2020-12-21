/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../EuclideanState.h"
#include "../ControlMatrix.h"
#include "../FeedbackMatrix.h"
#include "../StateControlMatrix.h"
#include "../StateMatrix.h"
#include "../ControlVector.h"

namespace ct {
namespace core {

template <typename SCALAR = double>
using ControlTrajectory = DiscreteTrajectory<ControlVector<SCALAR>, Eigen::aligned_allocator<ControlVector<SCALAR>>>;

template <typename SCALAR = double>
using ControlMatrixTrajectory = DiscreteTrajectory<ControlMatrix<SCALAR>>;

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using FeedbackTrajectory = DiscreteTrajectory<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>,
    Eigen::aligned_allocator<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>>;

template <int STATE_DIM, typename SCALAR = double>
using StateControlMatrixTrajectory = DiscreteTrajectory<StateControlMatrix<STATE_DIM, SCALAR>>;

template <int STATE_DIM, typename SCALAR = double>
using ControlStateMatrixTrajectory = DiscreteTrajectory<ControlStateMatrix<STATE_DIM, SCALAR>>;

template <int STATE_DIM, typename SCALAR = double>
using StateMatrixTrajectory = DiscreteTrajectory<StateMatrix<STATE_DIM, SCALAR>>;

template <int STATE_DIM, typename SCALAR = double>
using StateTrajectory =
    DiscreteTrajectory<StateVector<STATE_DIM, SCALAR>, Eigen::aligned_allocator<StateVector<STATE_DIM, SCALAR>>>;


}  // namespace core
}  // namespace ct
