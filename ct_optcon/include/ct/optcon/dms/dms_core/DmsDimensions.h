/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      Defines basic types used in the DMS algorithm
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DmsDimensions
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> state_matrix_array_t;

    typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t;
    typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_array_t;

    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;

    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef ct::core::ControlMatrixTrajectory<CONTROL_DIM, SCALAR> control_matrix_array_t;

    typedef SCALAR time_t;
    typedef ct::core::tpl::TimeArray<SCALAR> time_array_t;
};

}  // namespace optcon
}  // namespace ct
