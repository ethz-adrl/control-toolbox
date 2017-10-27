/**********************************************************************************************************************
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
**********************************************************************************************************************/

#include <ct/models/Quadrotor/quadrotor_dynamics/QuadrotorDynamics.hpp>

namespace ct {
namespace models {

using namespace quadrotor;

quadrotor::control_gain_matrix_t C_quadrotor(const quadrotor::state_vector_t &x, const quadrotor::control_vector_t &u)
{
    //	double qxQ = x(0);
    //	double qyQ = x(1);
    //	double qzQ = x(2);
    double qph = x(3);
    double qth = x(4);
    double qps = x(5);
    //	double dqxQ = x(6);
    //	double dqyQ = x(7);
    //	double dqzQ = x(8);
    //	double dqph = x(9);
    //	double dqth = x(10);
    //	double dqps = x(11);

    double t2, t3, t4, t5, t6, t7, t8;  // t9, t10, t11;

    t2 = 1.0 / mQ;
    t3 = cos(qth);
    t4 = 1.0 / Thxxyy;
    t5 = 1.0 / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    quadrotor::control_gain_matrix_t C;
    C.setZero();

    C(6, 0) = t2 * t8 * (1.0 / 2.0E1);
    C(7, 0) = t2 * t3 * sin(qph) * (-1.0 / 2.0E1);
    C(8, 0) = t2 * t3 * cos(qph) * (1.0 / 2.0E1);
    C(9, 1) = t4 * t5 * t7 * (1.0 / 2.0E1);
    C(9, 2) = t4 * t5 * t6 * (-1.0 / 2.0E1);
    C(10, 1) = t4 * t6 * (1.0 / 2.0E1);
    C(10, 2) = t4 * t7 * (1.0 / 2.0E1);
    C(11, 1) = t4 * t5 * t7 * t8 * (-1.0 / 2.0E1);
    C(11, 2) = t4 * t5 * t6 * t8 * (1.0 / 2.0E1);
    C(11, 3) = (1.0 / 2.0E1) / Thzz;

    return C;
}
}
}
