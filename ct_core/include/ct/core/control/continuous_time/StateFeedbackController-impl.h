/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::StateFeedbackController()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::StateFeedbackController(
    const StateVectorArray<STATE_DIM, SCALAR>& x_ref,
    const ControlVectorArray<CONTROL_DIM, SCALAR>& uff,
    const FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K,
    const SCALAR& deltaT,
    const SCALAR& t0,
    const InterpolationType& intType)
    : StateFeedbackBase(x_ref, uff, K, deltaT, t0, intType)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::StateFeedbackController(
    const StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other)
    : StateFeedbackBase(other) // x_ref_(other.x_ref_), uff_(other.uff_), K_(other.K_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::~StateFeedbackController()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>*
StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>::computeControl(
    const StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& t,
    ControlVector<CONTROL_DIM, SCALAR>& controlAction)
{
    controlAction = this->uff_.eval(t) + this->K_.eval(t) * (state - this->x_ref_.eval(t));
}

}  // core
}  // ct
