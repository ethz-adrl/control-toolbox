/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::SelectionMatrix(const SelectionMatrix& other)
    : Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>(other)
{
}

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::SelectionMatrix(bool floatingBase)
    : Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>()
{
    setIdentity(floatingBase);
}

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
template <typename T>
typename std::enable_if<(STATE_DIM >= 6), T>::type SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::setIdentity(
    bool floatingBase)
{
    this->setZero();

    if (floatingBase)
    {
        this->template bottomRightCorner<CONTROL_DIM, STATE_DIM - 6>().setIdentity();
    }
    else
    {
        this->template bottomRightCorner<CONTROL_DIM, CONTROL_DIM>().setIdentity();
    }
}

template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR>
template <typename T>
typename std::enable_if<(STATE_DIM < 6), T>::type SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::setIdentity(
    bool floatingBase)
{
    if (floatingBase)
        throw std::runtime_error("Selection Matrix for floating base systems should at least have 6 columns");

    this->setZero();
    this->template bottomRightCorner<CONTROL_DIM, CONTROL_DIM>().setIdentity();
}
}
}
