/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <ct/models/Quadrotor/quadrotor_dynamics/QuadrotorDynamics.hpp>

namespace ct {
namespace models {

using namespace quadrotor;

quadrotor::state_matrix_t A_quadrotor(const quadrotor::state_vector_t& x, const quadrotor::control_vector_t& u)
{
    double qph = x(3);
    double qth = x(4);
    double qps = x(5);
    double dqph = x(9);
    double dqth = x(10);
    double dqps = x(11);

    double Fz = u(0);
    double Mx = u(1);
    double My = u(2);

    double t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23, t24,
        t25;

    t2 = 1.0 / mQ;
    t3 = cos(qth);
    t4 = sin(qph);
    t5 = cos(qph);
    t6 = sin(qth);
    t7 = 1.0 / Thxxyy;
    t8 = cos(qps);
    t9 = sin(qps);
    t10 = 1.0 / t3;
    t11 = Thxxyy * 2.0;
    t12 = Thzz - t11;
    t13 = qth * 2.0;
    t14 = cos(t13);
    t15 = My * t9;
    t16 = sin(t13);
    t17 = 1.0 / (t3 * t3);
    t18 = qth * 3.0;
    t19 = sin(t18);
    t20 = My * t8;
    t21 = Mx * t9;
    t22 = t20 + t21;
    t23 = t3 * t3;
    t24 = t6 * t6;
    t25 = Thzz * dqps * t6;

    quadrotor::state_matrix_t A;
    A.setZero();

    A(0, 6) = 1.0;
    A(1, 7) = 1.0;
    A(2, 8) = 1.0;
    A(3, 9) = 1.0;
    A(4, 10) = 1.0;
    A(5, 11) = 1.0;
    A(6, 4) = Fz * t2 * t3;
    A(7, 3) = -Fz * t2 * t3 * t5;
    A(7, 4) = Fz * t2 * t4 * t6;
    A(8, 3) = -Fz * t2 * t3 * t4;
    A(8, 4) = -Fz * t2 * t5 * t6;
    A(9, 4) = -t6 * t7 * t17 *
                  (t15 - Mx * t8 + Thzz * dqps * dqth - Thxxyy * dqph * dqth * t6 * 2.0 + Thzz * dqph * dqth * t6) -
              dqph * dqth * t7 * t12;
    A(9, 5) = -t7 * t10 * t22;
    A(9, 9) = -dqth * t6 * t7 * t10 * t12;
    A(9, 10) = -t7 * t10 * (Thzz * dqps - Thxxyy * dqph * t6 * 2.0 + Thzz * dqph * t6);
    A(9, 11) = -Thzz * dqth * t7 * t10;
    A(10, 4) = -dqph * t7 * (t25 + Thxxyy * dqph * t14 - Thzz * dqph * t14);
    A(10, 5) = -t7 * (t15 - Mx * t8);
    A(10, 9) = t7 * (-Thxxyy * dqph * t16 + Thzz * dqps * t3 + Thzz * dqph * t16);
    A(10, 11) = Thzz * dqph * t3 * t7;
    A(11, 4) = t7 * t17 * (Mx * t8 * -4.0 + My * t9 * 4.0 + Thzz * dqps * dqth * 4.0 - Thxxyy * dqph * dqth * t6 * 9.0 -
                              Thxxyy * dqph * dqth * t19 + Thzz * dqph * dqth * t6 * 5.0 + Thzz * dqph * dqth * t19) *
               (1.0 / 4.0);
    A(11, 5) = t6 * t7 * t10 * t22;
    A(11, 9) = dqth * t7 * t10 * (Thzz - t11 + Thxxyy * t23 - Thzz * t23);
    A(11, 10) = t7 * t10 * (t25 - Thxxyy * dqph - Thxxyy * dqph * t24 + Thzz * dqph * t24);
    A(11, 11) = Thzz * dqth * t6 * t7 * t10;

    return A;
}
}
}
