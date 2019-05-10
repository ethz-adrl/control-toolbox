/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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

    template <typename T = void>  // do not use this template argument
    typename std::enable_if<(STATE_DIM < 6), T>::type setIdentity(bool floatingBase);

    template <typename T = void>  // do not use this template argument
    typename std::enable_if<(STATE_DIM >= 6), T>::type setIdentity(bool floatingBase);


private:
};
}
}
