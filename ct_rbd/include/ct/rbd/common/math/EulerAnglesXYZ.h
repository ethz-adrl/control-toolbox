
#pragma once

#include <Eigen/Dense>
#include "transformationHelpers.h"

namespace ct {
namespace rbd {
namespace tpl {

template <typename SCALAR>
class EulerAnglesXYZ : public Eigen::Matrix<SCALAR, 3, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = Eigen::Matrix<SCALAR, 3, 1>;
    using Matrix3 = Eigen::Matrix<SCALAR, 3, 3>;
    using Vector3 = Eigen::Matrix<SCALAR, 3, 1>;
    using QuaternionType = Eigen::Quaternion<SCALAR>;

    EulerAnglesXYZ() = default;
    virtual ~EulerAnglesXYZ() {}

    template <typename OtherDerived>
    EulerAnglesXYZ(const Eigen::MatrixBase<OtherDerived>& other) : BASE(other)
    {
    }

    template <typename OtherDerived>
    EulerAnglesXYZ& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->BASE::operator=(other);
        return *this;
    }

    EulerAnglesXYZ(const SCALAR& alpha, const SCALAR& beta, const SCALAR& gamma) : BASE(alpha, beta, gamma) {}

    EulerAnglesXYZ(const Matrix3& rotMat) { *this = fromRotationMatrix(rotMat); }

    EulerAnglesXYZ(const QuaternionType& q) { *this = fromQuaternion(q); }

    BASE& toImplementation() { return *this; }
    const BASE& toImplementation() const { return *this; }

    const Vector3& angles() const { return *this; }
    Vector3& angles() { return *this; }

    SCALAR alpha() const { return angles()(0); }
    SCALAR& alpha() { return angles()(0); }

    SCALAR beta() const { return angles()(1); }
    SCALAR& beta() { return angles()(1); }

    SCALAR gamma() const { return angles()(2); }
    SCALAR& gamma() { return angles()(2); }

    //! The Euler angles rotation inverse (which is as same as the negative)
    EulerAnglesXYZ inverse() const { return -*this; }

    static EulerAnglesXYZ fromRotationMatrix(const Matrix3& m)
    {
        EulerAnglesXYZ<SCALAR> e;
        e.angles() = m.eulerAngles(0, 1, 2);
        return e;
    }

    Matrix3 toRotationMatrix() const { return toQuaternion().toRotationMatrix(); }

    static EulerAnglesXYZ fromQuaternion(const QuaternionType& q) { return fromRotationMatrix(q.toRotationMatrix()); }

    QuaternionType toQuaternion() const
    {
        return Eigen::AngleAxis<SCALAR>(alpha(), Vector3::UnitX()) *
               Eigen::AngleAxis<SCALAR>(beta(), Vector3::UnitY()) * Eigen::AngleAxis<SCALAR>(gamma(), Vector3::UnitZ());
    }

    EulerAnglesXYZ& operator=(const Matrix3& m)
    {
        *this = fromRotationMatrix(m);
        return *this;
    }

    EulerAnglesXYZ& operator=(const QuaternionType& q)
    {
        *this = fromQuaternion(q);
        return *this;
    }

    operator QuaternionType() const
    {
        return Eigen::AngleAxis<SCALAR>(alpha(), Vector3::UnitX()) *
               Eigen::AngleAxis<SCALAR>(beta(), Vector3::UnitY()) * Eigen::AngleAxis<SCALAR>(gamma(), Vector3::UnitZ());
    }

    EulerAnglesXYZ getUnique(const SCALAR tol = 1e-3) const
    {
        EulerAnglesXYZ xyz(floatingPointModulo(alpha() + M_PI, 2 * M_PI) - M_PI,
            floatingPointModulo(beta() + M_PI, 2 * M_PI) - M_PI,
            floatingPointModulo(gamma() + M_PI, 2 * M_PI) - M_PI);  // wrap all angles into [-pi,pi)


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
    Eigen::Matrix<SCALAR, 3, 3> getMappingFromLocalAngularVelocityToDiff() const
    {
        using std::cos;
        using std::sin;
        Eigen::Matrix<SCALAR, 3, 3> mat = Eigen::Matrix<SCALAR, 3, 3>::Zero();
        const SCALAR x = alpha();
        const SCALAR y = beta();
        const SCALAR z = gamma();

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
};

}  // namespace tpl

using EulerAnglesXYZ = tpl::EulerAnglesXYZ<double>;

}  // namespace rbd
}  // namespace ct
