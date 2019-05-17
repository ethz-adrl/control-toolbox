/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Dense>

namespace ct {
namespace models {
namespace quadrotor {

const double pi = 3.14159265;

// mass / inertia
const double mQ = 0.546;         // mass of quadcopter [ kg ]
const double Thxxyy = 2.32e-3;   // moment of inertia around x,y [ kg*m^2 ]
const double Thzz = 3e-4;        // moment of inertia around z [ kg*m^2 ]
const double arm_len = 0.175;    // length of quadcopter arm [ m ]
const double grav_const = 9.81;  // gravitational constant [ m/s^2 ]

const double f_hover = mQ * grav_const;

// Thrust parameters
const double kF = 6.17092e-8 * 3600 / (2 * pi * 2 * pi);  // rotor thrust coefficient [ N/rad^2 ]
const double kM = 1.3167e-9 * 3600 / (2 * pi * 2 * pi);   // rotor moment coefficient [ Nm/rad^2]
const double wmax = 7800.0 * 2 * pi / 60;                 // maximum rotor speed [ rad/s ]
const double wmin = 1200.0 * 2 * pi / 60;                 // minimum rotor speed [ rad/s ]
const double Fsat_min = kF * wmin * wmin;
const double Fsat_max = kF * wmax * wmax;

const Eigen::Vector4d kFs(kF, kF, kF, kF);
const Eigen::Vector4d kMs(kM, kM, kM, kM);

}  // namespace quadrotor
}  // namespace models
}  // namespace ct
