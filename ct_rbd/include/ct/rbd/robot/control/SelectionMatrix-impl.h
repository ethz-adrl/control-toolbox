/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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
void SelectionMatrix<CONTROL_DIM, STATE_DIM, SCALAR>::setIdentity(bool floatingBase)
{
    this->setZero();

    if (floatingBase)
    {
        if (STATE_DIM < 6)
            throw std::runtime_error("Selection Matrix for floating base systems should at least have 6 columns");

        this->template bottomRightCorner<CONTROL_DIM, STATE_DIM - 6>().setIdentity();
    }
    else
    {
        this->template bottomRightCorner<CONTROL_DIM, CONTROL_DIM>().setIdentity();
    }
}
}
}
