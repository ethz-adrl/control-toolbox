/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

/**
 * \brief Selection Matrix for a Rigid Body Dynamics System
 *
 * This matrix maps control inputs to the states used in standard rigid body dynamics formulations such as
 *
 * \f[
 *  M \ddot{q} + C + G = S^T \tau + J_c \lambda
 * \f]
 *
 * where \f$ S \f$ is the selection matrix.
 */
template <size_t CONTROL_DIM, size_t STATE_DIM, typename SCALAR = double>
class SelectionMatrix : public Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SelectionMatrix() = delete;

    SelectionMatrix(const SelectionMatrix& other);

    SelectionMatrix(bool floatingBase);

    void setIdentity(bool floatingBase);

private:
};
}
}
