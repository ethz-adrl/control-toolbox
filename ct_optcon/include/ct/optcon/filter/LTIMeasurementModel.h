/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "LinearMeasurementModel.h"

namespace ct {
namespace optcon {

template <size_t OBS_DIM, size_t STATE_DIM, typename SCALAR = double>
class LTIMeasurementModel : public LinearMeasurementModel<OBS_DIM, STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = LinearMeasurementModel<OBS_DIM, STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;
    using typename Base::output_vector_t;
    using typename Base::output_matrix_t;
    using typename Base::output_state_matrix_t;
    using typename Base::Time_t;

    LTIMeasurementModel()
    {
        dHdx_.setZero();
        dHdw_.setIdentity();
    }

    LTIMeasurementModel(const output_state_matrix_t& C, const output_matrix_t& dHdw = output_matrix_t::Identity())
        : dHdx_(C)
    {
    }
    output_vector_t computeMeasurement(const state_vector_t& state, const Time_t& t = 0) override
    {
        return dHdx_ * state;
    }

    output_state_matrix_t computeDerivativeState(const state_vector_t& state, const Time_t& t) override
    {
        return dHdx_;
    }
    output_matrix_t computeDerivativeNoise(const state_vector_t& state, const Time_t& t) override { return dHdw_; }
protected:
    output_state_matrix_t dHdx_;
    output_matrix_t dHdw_;
};

}  // optcon
}  // ct
