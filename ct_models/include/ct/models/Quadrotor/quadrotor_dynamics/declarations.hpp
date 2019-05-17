/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace quadrotor {

const size_t nStates = 12;
const size_t nControls = 4;

typedef Eigen::Matrix<double, nStates, 1> state_vector_t;
typedef Eigen::Matrix<double, nControls, 1> control_vector_t;

typedef Eigen::Matrix<double, nStates, nStates> state_matrix_t;
typedef Eigen::Matrix<double, nControls, nControls> control_matrix_t;
typedef Eigen::Matrix<double, nStates, nControls> control_gain_matrix_t;
}
}
}
