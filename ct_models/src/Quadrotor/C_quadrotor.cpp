/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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
