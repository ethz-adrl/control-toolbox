/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/arrays/DiscreteArray.h>
#include <ct/core/types/arrays/TimeArray.h>

namespace ct {
namespace core {

template <int STATE_DIM, int CONTROL_DIM, class SCALAR = double>
class LinearFunctionMIMO
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void swap(LinearFunctionMIMO& arg)
    {
        uff_.swap(arg.uff_);
        deltaUff_.swap(arg.deltaUff_);
        k_.swap(arg.k_);
    }

    void setZero()
    {
        uff_.setZero();
        deltaUff_.setZero();
        k_.setZero();
    }

    TimeArray time_;
    DiscreteArray<ct::core::ControlVector<CONTROL_DIM, SCALAR>> uff_;
    DiscreteArray<ct::core::ControlVector<CONTROL_DIM, SCALAR>> deltaUff_;
    DiscreteArray<Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>> k_;
};


template <int STATE_DIM, int DIM1, int DIM2, class SCALAR = double>
class GeneralLinearFunction
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void swap(GeneralLinearFunction& arg)
    {
        uff_.swap(arg.uff_);
        deltaUff_.swap(arg.deltaUff_);
        k_.swap(arg.k_);
    }

    void setZero()
    {
        uff_.setZero();
        deltaUff_.setZero();
        k_.setZero();
    }

    TimeArray time_;
    DiscreteArray<Eigen::Matrix<SCALAR, DIM1, DIM2>> uff_;
    DiscreteArray<Eigen::Matrix<SCALAR, DIM1, DIM2>> deltaUff_;
    DiscreteArray<Eigen::Matrix<SCALAR, DIM1, STATE_DIM>> k_;
};

}  // core
}  // ct
