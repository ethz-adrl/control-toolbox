/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {


/*! \brief Floating-point modulo
 *
 * The result (the remainder) has same sign as the divisor.
 * Similar to matlab's mod(); Not similar to fmod():    floatingPointModulo(-3,4)= 1   fmod(-3,4)= -3
 * 
 * This method is taken from kindr, https://github.com/ANYbotics/kindr/blob/d16112787cdbc2ff798d44e92501e923e5856159/include/kindr/common/common.hpp
 * Copyright by Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 */
template <typename T>
T floatingPointModulo(T x, T y)
{
    static_assert(!std::numeric_limits<T>::is_exact, "floatingPointModulo: floating-point type expected");

    if (y == 0.0)
        return x;

    double m = x - y * floor(x / y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)  // modulo range: [0..y)
    {
        if (m >= y)  // mod(-1e-16, 360.    ): m= 360.
            return 0;

        if (m < 0)
        {
            if (y + m == y)
                return 0;  // just in case...
            else
                return y + m;  // Mod(106.81415022205296 , 2*M_PI ): m= -1.421e-14
        }
    }
    else  // modulo range: (y..0]
    {
        if (m <= y)  // mod(1e-16, -360.   ): m= -360.
            return 0;

        if (m > 0)
        {
            if (y + m == y)
                return 0;  // just in case...
            else
                return y + m;  // mod(-106.81415022205296, -2*M_PI): m= 1.421e-14
        }
    }

    return m;
}


template <typename SCALAR>
Eigen::EulerAngles<SCALAR, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> getUnqiueEulerAnglesXYZ(
    const Eigen::EulerAngles<SCALAR, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> eulerIn)
{
    Eigen::EulerAngles<SCALAR, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> xyz(
        floatingPointModulo(eulerIn.alpha() + M_PI, 2 * M_PI) - M_PI,
        floatingPointModulo(eulerIn.beta() + M_PI, 2 * M_PI) - M_PI,
        floatingPointModulo(eulerIn.gamma() + M_PI, 2 * M_PI) - M_PI);  // wrap all angles into [-pi,pi)

    const SCALAR tol = 1e-3;

    // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    if (xyz.beta() < -M_PI / 2 - tol)
    {
        if (xyz.alpha() < 0)
        {
            xyz.alpha() = xyz.alpha() + M_PI;
        }
        else
        {
            xyz.alpha() = xyz.alpha() - M_PI;
        }

        xyz.beta() = -(xyz.beta() + M_PI);

        if (xyz.gamma() < 0)
        {
            xyz.gamma() = xyz.gamma() + M_PI;
        }
        else
        {
            xyz.gamma() = xyz.gamma() - M_PI;
        }
    }
    else if (-M_PI / 2 - tol <= xyz.beta() && xyz.beta() <= -M_PI / 2 + tol)
    {
        xyz.alpha() -= xyz.gamma();
        xyz.gamma() = 0;
    }
    else if (-M_PI / 2 + tol < xyz.beta() && xyz.beta() < M_PI / 2 - tol)
    {
        // ok
    }
    else if (M_PI / 2 - tol <= xyz.beta() && xyz.beta() <= M_PI / 2 + tol)
    {
        // todo: M_PI/2 should not be in range, other formula?
        xyz.alpha() += xyz.gamma();
        xyz.gamma() = 0;
    }
    else  // M_PI/2 + tol < xyz.beta()
    {
        if (xyz.alpha() < 0)
        {
            xyz.alpha() = xyz.alpha() + M_PI;
        }
        else
        {
            xyz.alpha() = xyz.alpha() - M_PI;
        }

        xyz.beta() = -(xyz.beta() - M_PI);

        if (xyz.gamma() < 0)
        {
            xyz.gamma() = xyz.gamma() + M_PI;
        }
        else
        {
            xyz.gamma() = xyz.gamma() - M_PI;
        }
    }

    return xyz;
}


// todo we should use the AD traits here!!
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 3> getMappingFromLocalAngularVelocityToDiff(
    const Eigen::EulerAngles<SCALAR, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> eulerIn)
{
    using std::cos;
    using std::sin;
    Eigen::Matrix<SCALAR, 3, 3> mat = Eigen::Matrix<SCALAR, 3, 3>::Zero();
    const SCALAR x = eulerIn.alpha();
    const SCALAR y = eulerIn.beta();
    const SCALAR z = eulerIn.gamma();

    const SCALAR t2 = cos(y);
    const SCALAR t3 = 1.0 / t2;
    const SCALAR t4 = sin(z);
    const SCALAR t5 = cos(z);
    const SCALAR t6 = sin(y);
    mat(0, 0) = t3 * t5;
    mat(0, 1) = -t3 * t4;
    mat(1, 0) = t4;
    mat(1, 1) = t5;
    mat(2, 0) = -t3 * t5 * t6;
    mat(2, 1) = t3 * t4 * t6;
    mat(2, 2) = 1.0;

    return mat;
}


/*! \brief Returns the local quaternion diff matrix HBar: LocalAngularVelocity = 2*HBar*qdiff, qdiff = 0.5*HBar^T*LocalAngularVelocity
   *  \returns the local quaternion diff matrix HBar
   */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 4> getLocalQuaternionDiffMatrix(const Eigen::Quaternion<SCALAR>& q)
{
    Eigen::Matrix<SCALAR, 3, 4> HBar;
    HBar(0, 0) = -q.x();
    HBar(0, 1) = q.w();
    HBar(0, 2) = q.z();
    HBar(0, 3) = -q.y();
    HBar(1, 0) = -q.y();
    HBar(1, 1) = -q.z();
    HBar(1, 2) = q.w();
    HBar(1, 3) = q.x();
    HBar(2, 0) = -q.z();
    HBar(2, 1) = q.y();
    HBar(2, 2) = -q.x();
    HBar(2, 3) = q.w();
    return HBar;
}

}  // namespace rbd
}  // namespace ct
