/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SystemModelBase
{
public:
    using state_vector_t   = ct::core::StateVector<STATE_DIM, SCALAR>;
    using state_matrix_t   = ct::core::StateMatrix<STATE_DIM, SCALAR>;
    using control_vector_t = ct::core::ControlVector<CONTROL_DIM, SCALAR>;
    using Time_t           = ct::core::Time;

    virtual ~SystemModelBase() {}
    virtual state_vector_t computeDynamics(
        const state_vector_t& state,
        const control_vector_t& control,
        Time_t t) = 0;

    virtual state_matrix_t computeDerivativeState(
        const state_vector_t& state,
        const control_vector_t& control,
        Time_t t) = 0;
    virtual state_matrix_t computeDerivativeNoise(
        const state_vector_t& state,
        const control_vector_t& control,
        Time_t t) = 0;
};

}  // optcon
}  // ct
