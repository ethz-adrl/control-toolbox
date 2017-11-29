/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <cmath>

#include <ct/models/Quadrotor/quadrotor_definitions/quadModelParameters.hpp>
#include <ct/models/Quadrotor/quadrotor_dynamics/declarations.hpp>

namespace ct {
namespace models {

quadrotor::state_matrix_t A_quadrotor(const quadrotor::state_vector_t &x, const quadrotor::control_vector_t &u);
quadrotor::control_gain_matrix_t B_quadrotor(const quadrotor::state_vector_t &x, const quadrotor::control_vector_t &u);
quadrotor::state_vector_t quadrotor_ode(const quadrotor::state_vector_t &x, const quadrotor::control_vector_t &u);

}  // models
}  // ct
