/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef CT_MODELS_QUADVISPARAMETERS_HPP_
#define CT_MODELS_QUADVISPARAMETERS_HPP_

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


#endif /* QUADVISPARAMETERS_HPP_ */
