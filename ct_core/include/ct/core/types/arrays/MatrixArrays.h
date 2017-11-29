/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "../ControlMatrix.h"
#include "../FeedbackMatrix.h"
#include "../StateControlMatrix.h"
#include "../StateMatrix.h"
#include "../StateVector.h"
#include "../ControlVector.h"

#include "DiscreteArray.h"

namespace ct {
namespace core {

template <size_t CONTROL_DIM, typename SCALAR = double>
using ControlMatrixArray = DiscreteArray<ControlMatrix<CONTROL_DIM, SCALAR>>;

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using FeedbackArray = DiscreteArray<FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>;

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
using StateControlMatrixArray = DiscreteArray<StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>>;

template <size_t STATE_DIM, typename SCALAR = double>
using StateMatrixArray = DiscreteArray<StateMatrix<STATE_DIM, SCALAR>>;

template <size_t STATE_DIM, typename SCALAR = double>
using StateVectorArray = DiscreteArray<StateVector<STATE_DIM, SCALAR>>;

template <size_t CONTROL_DIM, typename SCALAR = double>
using ControlVectorArray = DiscreteArray<ControlVector<CONTROL_DIM, SCALAR>>;
}
}
