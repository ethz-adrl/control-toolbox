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


#include <ct/models/Quadrotor/quadrotor_dynamics/QuadrotorDynamics.hpp>

namespace ct {
namespace models {

using namespace quadrotor;


quadrotor::control_gain_matrix_t B_quadrotor(const quadrotor::state_vector_t &x, const quadrotor::control_vector_t &u)
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

    //	double u1 = u(0);
    //	double u2 = u(1);
    //	double u3 = u(2);
    //	double u4 = u(3);
    //
    //	double kF1 = kFs(0);
    //	double kF2 = kFs(1);
    //	double kF3 = kFs(2);
    //	double kF4 = kFs(3);
    //
    //	double kM1 = kMs(0);
    //	double kM2 = kMs(1);
    //	double kM3 = kMs(2);
    //	double kM4 = kMs(3);

    double t2, t3, t4, t5, t6, t7, t8;  // t9, t10, t11;

    t2 = 1.0 / mQ;
    t3 = cos(qth);
    t4 = 1.0 / Thxxyy;
    t5 = 1.0 / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    quadrotor::control_gain_matrix_t B;
    B.setZero();

    B(6, 0) = t2 * t8;
    B(7, 0) = -t2 * t3 * sin(qph);
    B(8, 0) = t2 * t3 * cos(qph);
    B(9, 1) = t4 * t5 * t7;
    B(9, 2) = -t4 * t5 * t6;
    B(10, 1) = t4 * t6;
    B(10, 2) = t4 * t7;
    B(11, 1) = -t4 * t5 * t7 * t8;
    B(11, 2) = t4 * t5 * t6 * t8;
    B(11, 3) = 1.0 / Thzz;

    return B;
}
}
}
