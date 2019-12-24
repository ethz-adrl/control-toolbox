/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#include <kindr/Core>
#pragma GCC diagnostic pop

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \brief Pose of a single rigid body
 *
 * \ingroup State
 *
 * This implements the pose of a single rigid body. Internally, the pose is stored as
 * either a quaternion (default) or Euler angles. Hence, the user is encouraged to use
 * rotateBaseToInertia() or rotateInertiaToBase() instead of directly working with either
 * representation. This will prevent unnecessary conversions.
 */
template <typename SCALAR = double>
class RigidBodyPose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * \brief the orientation can either be stored as EulerAngles or as quaternion.
     * \todo reduce to quaternion only
     */
    enum STORAGE_TYPE
    {
        QUAT = 0,
        EULER = 1
    };

    using HomogeneousTransform = kindr::HomogeneousTransformationPosition3RotationQuaternion<SCALAR>;
    using Matrix4Tpl = Eigen::Matrix<SCALAR, 4, 4>;
    using Matrix3Tpl = Eigen::Matrix<SCALAR, 3, 3>;
    using Position3Tpl = kindr::Position<SCALAR, 3>;
    using Vector3Tpl = Eigen::Matrix<SCALAR, 3, 1>;

    //! default construct a RigidBodyPose
    RigidBodyPose(STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0))
    {
    }

    //! construct a RigidBodyPose from Euler Angles and a position vector
    RigidBodyPose(const kindr::EulerAnglesXyz<SCALAR>& orientationEulerXyz,
        const Position3Tpl& position,
        STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),
          position_(position)
    {
        setFromEulerAnglesXyz(orientationEulerXyz);
    }

    //! construct a RigidBodyPose from a rotation quaternion and a position vector
    RigidBodyPose(const kindr::RotationQuaternion<SCALAR>& orientationQuat,
        const Position3Tpl& position,
        STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),
          position_(position)
    {
        setFromRotationQuaternion(orientationQuat);
    }

    //! construct a RigidBodyPose from a rotation quaternion and a position vector
    RigidBodyPose(const Eigen::Quaternion<SCALAR>& orientationQuat,
        const Eigen::Matrix<SCALAR, 3, 1>& position,
        STORAGE_TYPE storage = EULER)
        : RigidBodyPose(kindr::RotationQuaternion<SCALAR>(orientationQuat), Position3Tpl(position))
    {
    }

    //! construct a RigidBodyPose from a homogeneous transformation matrix
    RigidBodyPose(const Matrix4Tpl& homTransform, STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),
          position_(homTransform.template topRightCorner<3, 1>())
    {
        kindr::RotationMatrix<SCALAR> rotMat(homTransform.template topLeftCorner<3, 3>());
        setFromRotationMatrix(rotMat);
    }

    //! copy-constructor for a RigidBodyPose
    RigidBodyPose(const RigidBodyPose<SCALAR>& arg)
        : storage_(arg.storage_), quat_(arg.quat_), euler_(arg.euler_), position_(arg.position_)
    {
    }

    //! destructor for a rigid body pose
    ~RigidBodyPose() = default;

    inline bool isNear(const RigidBodyPose& rhs, const double& tol = 1e-10) const
    {
        return getRotationQuaternion().isNear(rhs.getRotationQuaternion(), tol) &&
               position().toImplementation().isApprox(rhs.position().toImplementation(), tol);
    }

    /**
     * \brief This method returns the Euler angles rotation (X,Y,Z / roll,pitch,yaw).
     */
    kindr::EulerAnglesXyz<SCALAR> getEulerAnglesXyz() const
    {
        if (storedAsEuler())
        {
            return euler_;
        }
        else
        {
            return kindr::EulerAnglesXyz<SCALAR>(quat_);
        }
    }

    /**
     * \brief This method returns the quaternion rotation.
     */
    kindr::RotationQuaternion<SCALAR> getRotationQuaternion() const
    {
        if (storedAsEuler())
        {
            return kindr::RotationQuaternion<SCALAR>(euler_);
        }
        else
        {
            return quat_;
        }
    }

    /**
     * \brief This method returns the  rotation matrix from base to inertia frame. This method returns
     * \f$ _iR_b \f$ which is the following rotation matrix \f$ _iv = {_iR_b} {_bv} \f$.
     */
    kindr::RotationMatrix<SCALAR> getRotationMatrix() const
    {
        if (storedAsEuler())
        {
            Eigen::AngleAxis<SCALAR> rollAngle(euler_.toImplementation()(0), Eigen::Matrix<SCALAR, 3, 1>::UnitX());
            Eigen::AngleAxis<SCALAR> pitchAngle(euler_.toImplementation()(1), Eigen::Matrix<SCALAR, 3, 1>::UnitY());
            Eigen::AngleAxis<SCALAR> yawAngle(euler_.toImplementation()(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ());
            Eigen::Quaternion<SCALAR> q = rollAngle * pitchAngle * yawAngle;
            return kindr::RotationMatrix<SCALAR>(q.matrix());

            //            return kindr::RotationMatrix<SCALAR>(euler_); // this is not JIT compatible as it uses a unit quat temporarily
        }
        else
        {
            return kindr::RotationMatrix<SCALAR>(quat_);
        }
    }

    /**
     * \brief This method sets the Euler angles rotation (X,Y,Z / roll,pitch,yaw) from a kindr type.
     */
    void setFromEulerAnglesXyz(const kindr::EulerAnglesXyz<SCALAR>& eulerAngles)
    {
        if (storedAsEuler())
        {
            euler_.toImplementation() = eulerAngles.toImplementation();
        }
        else
        {
            quat_ = eulerAngles;
        }
    }

    /**
     * \brief This method sets the Euler angles rotation (X,Y,Z / roll,pitch,yaw) from a kindr type.
     */
    void setFromEulerAnglesXyz(const Vector3Tpl& eulerAngles)
    {
        if (storedAsEuler())
        {
            euler_.toImplementation() = eulerAngles;
        }
        else
        {
            quat_ = kindr::EulerAnglesXyz<SCALAR>(eulerAngles);
        }
    }

    /**
     * \brief This method sets the quaternion angles rotation (X,Y,Z / roll,pitch,yaw) from a kindr quaternion type.
     */
    void setFromRotationQuaternion(const kindr::RotationQuaternion<SCALAR>& quat)
    {
        if (storedAsEuler())
        {
            euler_ = quat;
        }
        else
        {
            quat_ = quat;
        }
    }

    /**
     * \brief This method sets the quaternion angles rotation (X,Y,Z / roll,pitch,yaw) from an Eigen quaternion
     */
    void setFromRotationQuaternion(const Eigen::Quaterniond& quat)
    {
        if (storedAsEuler())
        {
            euler_ = kindr::EulerAnglesXyz<SCALAR>(kindr::RotationQuaternion<SCALAR>(quat));
        }
        else
        {
            quat_ = kindr::RotationQuaternion<SCALAR>(quat);
        }
    }

    /**
     * \brief This method sets the quaternion angles rotation from a kindr rotation matrix
     */
    void setFromRotationMatrix(const kindr::RotationMatrix<SCALAR>& rotMat)
    {
        if (storedAsEuler())
        {
            euler_ = kindr::EulerAnglesXyz<SCALAR>(rotMat);
        }
        else
        {
            quat_ = kindr::RotationQuaternion<SCALAR>(rotMat);
        }
    }

    /**
     * \brief This method returns the position of the Base frame in the inertia frame.
     */
    const Position3Tpl& position() const { return position_; }

    /**
     * \brief This method returns the position of the Base frame in the inertia frame.
     */
    Position3Tpl& position() { return position_; }

    /**
     * \brief This method returns the Base frame in a custom Frame specified in the Inertia Frame.
     */
    RigidBodyPose<SCALAR> inReferenceFrame(const RigidBodyPose<SCALAR>& ref_frame) const
    {
        if (ref_frame.storedAsEuler() && storedAsEuler())
        {
            return RigidBodyPose<SCALAR>(ref_frame.getEulerAnglesXyz().inverted() * euler_,
                ref_frame.getEulerAnglesXyz().inverseRotate(position() - ref_frame.position()));
        }
        else if (ref_frame.storedAsEuler() && !storedAsEuler())
        {
            return RigidBodyPose<SCALAR>(ref_frame.getEulerAnglesXyz().inverted() * quat_,
                ref_frame.getEulerAnglesXyz().inverseRotate(position() - ref_frame.position()));
        }
        else if (!ref_frame.storedAsEuler() && storedAsEuler())
        {
            return RigidBodyPose<SCALAR>(ref_frame.getRotationQuaternion().inverted() * euler_,
                ref_frame.getRotationQuaternion().inverseRotate(position() - ref_frame.position()));
        }
        else
        {
            return RigidBodyPose<SCALAR>(ref_frame.getRotationQuaternion().inverted() * quat_,
                ref_frame.getRotationQuaternion().inverseRotate(position() - ref_frame.position()));
        }
    }

    /**
     * \brief This methods rotates a 3D vector expressed in Base frame to Inertia Frame.
     */
    template <class Vector3s>
    Vector3s rotateBaseToInertia(const Vector3s& vector) const
    {
        Vector3s out;

        if (storedAsEuler())
        {
            kindr::RotationMatrix<SCALAR> rotationMatrix = getRotationMatrix();

            // incredibly ugly hack to get along with different types
            Eigen::Matrix<SCALAR, 3, 1> vec_temp;
            vec_temp << vector(0), vector(1), vector(2);

            Eigen::Matrix<SCALAR, 3, 1> result = rotationMatrix.toImplementation() * vec_temp;
            out = (Vector3s)result;

            //            return euler_.rotate(vector); // temporarily replaced -- the kindr rotate() method is not auto-diffable
        }
        else
        {
            out = quat_.rotate(vector);
        }

        return out;
    }


    /**
     * \brief This methods rotates a 3D matrix expressed in Base frame to Inertia Frame.
     */
    Matrix3Tpl rotateBaseToInertiaMat(const Matrix3Tpl& mat) const
    {
        kindr::RotationMatrix<SCALAR> rotMat = getRotationMatrix();
        return rotMat.toImplementation() * mat;
    }


    /**
     * \brief This methods rotates a quaternion expressed in Base frame to Inertia Frame.
     */
    kindr::RotationQuaternion<SCALAR> rotateBaseToInertiaQuaternion(const kindr::RotationQuaternion<SCALAR>& q) const
    {
        kindr::RotationQuaternion<SCALAR> result;

        if (storedAsEuler())
            result = kindr::RotationQuaternion<SCALAR>(euler_) * q;
        else
            result = quat_ * q;

        return result;
    }

    /**
     * \brief This methods rotates a 3D vector expressed in Inertia frame to Base Frame.
     */
    template <class Vector3s>
    Vector3s rotateInertiaToBase(const Vector3s& vector) const
    {
        // more efficient than inverseRotate and (more importantly) compatible with auto-diff
        // https://github.com/ethz-asl/kindr/issues/84
        // return Vector3d::Zero();
        // return kindr::RotationMatrix<SCALAR>(euler_).transposed().rotate(vector);
        if (storedAsEuler())
        {
            return kindr::RotationMatrix<SCALAR>(euler_).transposed().rotate(vector);
        }
        else
        {
            return quat_.inverseRotate(vector);
        }
    }

    /**
     * \brief This methods returns the Homogeneous transform from the Base frame to the inertia frame.
     */
    HomogeneousTransform getHomogeneousTransform() const
    {
        throw std::runtime_error("get homogeneous transform not implemented");
        return HomogeneousTransform();
    }

    /**
     * Returns gravity vector in world (0.0, 0.0, -9.81)
     * @return
     */
    static Vector3Tpl gravity3DW(SCALAR g = SCALAR(-9.81)) { return Vector3Tpl(SCALAR(0.0), SCALAR(0.0), g); }
    /**
     * \brief This methods returns the 3D gravity vector expressed in the Base frame.
     */
    Vector3Tpl computeGravityB(const Vector3Tpl& gravityW = gravity3DW()) const
    {
        return rotateInertiaToBase(gravityW);
    }

    /**
     * \brief This methods returns the 6D gravity vector expressed in the Base frame.
     */
    Eigen::Matrix<SCALAR, 6, 1> computeGravityB6D(const Vector3Tpl& gravityW = gravity3DW()) const
    {
        Eigen::Matrix<SCALAR, 6, 1> gravityWout(Eigen::Matrix<SCALAR, 6, 1>::Zero());
        gravityWout.template tail<3>() = rotateInertiaToBase(gravityW);
        return gravityWout;
    }

    /**
     * \brief Sets orientation to identity and position to (0, 0, 0)
     */
    void setIdentity()
    {
        euler_.setIdentity();
        quat_.setIdentity();
        position().setZero();
    }

    void setRandom()
    {
        euler_.setRandom();
        quat_ = euler_;
        position().toImplementation().setRandom();
    }

    STORAGE_TYPE getStorageType() const { return storage_; }

private:
    bool storedAsEuler() const
    {
        if (storage_ == EULER)
            return true;
        else
            return false;
    }

    STORAGE_TYPE storage_;

    kindr::RotationQuaternion<SCALAR> quat_;
    kindr::EulerAnglesXyz<SCALAR> euler_;

    Position3Tpl position_;
};

}  // namespace tpl

// convenience typedef (required)
typedef tpl::RigidBodyPose<double> RigidBodyPose;

}  // namespace rbd
}  // namespace ct
