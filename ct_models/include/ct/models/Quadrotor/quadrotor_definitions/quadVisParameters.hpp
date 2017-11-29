/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <quadrotor_definitions/quadModelParameters.hpp>
#include <Eigen/Dense>

namespace ct {
namespace models {
namespace quadrotor {

// A few parameters relevant for the visualization only:

enum ROTORS
{
    ROTOR_1 = 0,
    ROTOR_2 = 1,
    ROTOR_3 = 2,
    ROTOR_4 = 3,
    N_ROTORS
};

const double arm_width = arm_len / 8.0;
const double rotorDiameter = 2.0 * 0.1;
const double rotorHeight = rotorDiameter * 0.1;

const Eigen::Vector3d rotorPositions[N_ROTORS] = {Eigen::Vector3d(arm_len, 0.0, arm_width / 2.0),
    Eigen::Vector3d(0.0, arm_len, arm_width / 2.0), Eigen::Vector3d(-arm_len, 0.0, arm_width / 2.0),
    Eigen::Vector3d(0.0, -arm_len, arm_width / 2.0)};

}  // namespace quadrotor
}
}
