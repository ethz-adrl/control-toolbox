/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../ControlMatrix.h"
#include "../FeedbackMatrix.h"
#include "../StateControlMatrix.h"
#include "../StateMatrix.h"
#include "../StateVector.h"
#include "../ControlVector.h"

#include "DiscreteTrajectoryBase.h"

namespace ct {
namespace core {

template <size_t CONTROL_DIM, typename SCALAR = double, typename TIME_SCALAR = SCALAR>
using ControlTrajectory = DiscreteTrajectoryBase<ControlVector<CONTROL_DIM, SCALAR>,
    Eigen::aligned_allocator<ControlVector<CONTROL_DIM, SCALAR>>,
    TIME_SCALAR>;

template <size_t CONTROL_DIM, typename SCALAR = double>
using ControlMatrixTrajectory = DiscreteTrajectoryBase<ControlMatrix<CONTROL_DIM, SCALAR>>;

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using FeedbackTrajectory = DiscreteTrajectoryBase<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>,
    Eigen::aligned_allocator<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>,
    SCALAR>;

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using StateControlMatrixTrajectory = DiscreteTrajectoryBase<StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>;

template <size_t STATE_DIM, typename SCALAR = double>
using StateMatrixTrajectory = DiscreteTrajectoryBase<StateMatrix<STATE_DIM, SCALAR>>;

template <size_t STATE_DIM, typename SCALAR = double, typename TIME_SCALAR = SCALAR>
using StateTrajectory = DiscreteTrajectoryBase<StateVector<STATE_DIM, SCALAR>,
    Eigen::aligned_allocator<StateVector<STATE_DIM, SCALAR>>,
    TIME_SCALAR>;


}  // core
}  // ct
