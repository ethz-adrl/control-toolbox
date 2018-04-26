/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


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
