/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/common/math/EulerAnglesXYZ.h>
#include <ct/rbd/common/math/transformationHelpers.h>

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

    using Matrix4Tpl = Eigen::Matrix<SCALAR, 4, 4>;
    using Matrix3Tpl = Eigen::Matrix<SCALAR, 3, 3>;
    using Vector3Tpl = Eigen::Matrix<SCALAR, 3, 1>;
    using Position3Tpl = Vector3Tpl;
    using RotationMatrix = Matrix3Tpl;
    using EulerAnglesXYZTpl = tpl::EulerAnglesXYZ<SCALAR>;
    using QuaternionTpl = Eigen::Quaternion<SCALAR>;

    //! default construct a RigidBodyPose
    RigidBodyPose(STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0))
    {
    }

    //! construct a RigidBodyPose from Euler Angles and a position vector
    RigidBodyPose(const EulerAnglesXYZTpl& orientationEulerXyz,
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
    RigidBodyPose(const QuaternionTpl& orientationQuat, const Position3Tpl& position, STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),
          position_(position)
    {
        setFromRotationQuaternion(orientationQuat);
    }

    //! construct a RigidBodyPose from a homogeneous transformation matrix
    RigidBodyPose(const Matrix4Tpl& homTransform, STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),
          position_(homTransform.template topRightCorner<3, 1>())
    {
        RotationMatrix rotMat(homTransform.template topLeftCorner<3, 3>());
        setFromRotationMatrix(rotMat);
    }

    //! copy-constructor for a RigidBodyPose
    RigidBodyPose(const RigidBodyPose<SCALAR>& arg)
        : storage_(arg.storage_), quat_(arg.quat_), euler_(arg.euler_), position_(arg.position_)
    {
    }

    ~RigidBodyPose() = default;


    bool isApprox(const RigidBodyPose& rhs, const double& tol = 1e-10) const { return isNear(rhs, tol); }

    bool isNear(const RigidBodyPose& rhs, const double& tol = 1e-10) const
    {
        QuaternionTpl qNeg;
        qNeg.coeffs() = -rhs.getRotationQuaternion().coeffs();
        bool rotOK = getRotationQuaternion().isApprox(rhs.getRotationQuaternion(), tol) ||
                     getRotationQuaternion().isApprox(qNeg, tol);

        bool transOK = position().isApprox(rhs.position(), tol);
        return rotOK && transOK;
    }

    /**
     * \brief This method returns the Euler angles rotation (X,Y,Z / roll,pitch,yaw).
     */
    EulerAnglesXYZTpl getEulerAnglesXyz() const
    {
        if (storedAsEuler())
            return euler_;
        else
            return EulerAnglesXYZTpl(quat_);
    }

    /**
     * \brief This method returns the quaternion rotation.
     */
    QuaternionTpl getRotationQuaternion() const
    {
        if (storedAsEuler())
            return euler_.toQuaternion();
        else
            return quat_;
    }

    /**
     * \brief This method returns the  rotation matrix from base to inertia frame. This method returns
     * \f$ _iR_b \f$ which is the following rotation matrix \f$ _iv = {_iR_b} {_bv} \f$.
     */
    RotationMatrix getRotationMatrix() const
    {
        if (storedAsEuler())
            return euler_.toRotationMatrix();
        else
            return quat_.toRotationMatrix();
    }

    /**
     * \brief This method sets the Euler angles rotation (X,Y,Z / roll,pitch,yaw)
     */
    void setFromEulerAnglesXyz(const EulerAnglesXYZTpl& eulerAngles)
    {
        if (storedAsEuler())
            euler_ = eulerAngles;
        else
            quat_ = eulerAngles.toQuaternion();
    }

    /**
     * \brief This method sets the Euler angles rotation (X,Y,Z / roll,pitch,yaw)
     */
    void setFromEulerAnglesXyz(const SCALAR a, const SCALAR b, const SCALAR c)
    {
        EulerAnglesXYZTpl eulerAngles(a, b, c);
        setFromEulerAnglesXyz(eulerAngles);
    }


    /**
     * \brief This method sets the quaternion angles rotation (X,Y,Z / roll,pitch,yaw) from an Eigen quaternion
     */
    void setFromRotationQuaternion(const QuaternionTpl& quat)
    {
        if (storedAsEuler())
            euler_ = EulerAnglesXYZTpl(quat);
        else
            quat_ = quat;
    }

    /**
     * \brief This method sets the quaternion angles rotation from a rotation matrix
     */
    void setFromRotationMatrix(const RotationMatrix& rotMat)
    {
        if (storedAsEuler())
            euler_ = EulerAnglesXYZTpl(rotMat);
        else
            quat_ = QuaternionTpl(rotMat);
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
        throw std::runtime_error("inReferenceFrame() is obviously wrong.");
        /*
        if (ref_frame.storedAsEuler() && storedAsEuler())
        {
            auto rot_m = ref_frame.getRotationMatrix().transpose() * getRotationMatrix().toImplementation();
            auto rot = rot_m.eulerAngles(0,1,2);

            auto v = (ref_frame.getRotationMatrix().toImplementation().transpose()) *
                       (position().toImplementation() - ref_frame.position().toImplementation());

            auto rot_kindr = ref_frame.getEulerAnglesXyz().inverted() * euler_;
            auto v_kindr = ref_frame.getEulerAnglesXyz().inverseRotate(position() - ref_frame.position());


            std::cout << rot << std::endl;
            std::cout << rot_kindr.toImplementation() << std::endl;
            std::cout << v << std::endl;
            std::cout << v_kindr.toImplementation() << std::endl;

            assert(rot.isApprox(rot_kindr.toImplementation()));
            assert(v.isApprox(v_kindr.toImplementation()));

            RigidBodyPose<SCALAR> rbd_pose(rot_kindr, v_kindr);

            return rbd_pose;
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
        else if (!ref_frame.storedAsEuler() && !storedAsEuler())
        {
            return RigidBodyPose<SCALAR>(ref_frame.getRotationQuaternion().inverted() * quat_,
                ref_frame.getRotationQuaternion().inverseRotate(position() - ref_frame.position()));
        }
        else
            throw std::runtime_error("Invalid Pose storage type in both poses in inReferenceFrame()");
    */
    }

    /**
     * \brief This methods rotates a 3D vector expressed in Base frame to Inertia Frame.
     */
    template <class Vector3s>
    Vector3Tpl rotateBaseToInertia(const Vector3s& vector) const
    {
        if (storedAsEuler())
            return getRotationMatrix() * vector;
        else
            return quat_ * vector;
    }


    /**
     * \brief This methods rotates a 3D matrix expressed in Base frame to Inertia Frame.
     */
    Matrix3Tpl rotateBaseToInertiaMat(const Matrix3Tpl& mat) const
    {
        RotationMatrix rotMat = getRotationMatrix();
        return rotMat * mat;
    }


    /**
     * \brief This methods rotates a quaternion expressed in Base frame to Inertia Frame.
     */
    QuaternionTpl rotateBaseToInertiaQuaternion(const QuaternionTpl& q) const
    {
        if (storedAsEuler())
            return euler_.toQuaternion() * q;
        else
            return quat_ * q;
    }

    /**
     * \brief This methods rotates a 3D vector expressed in Inertia frame to Base Frame.
     */
    template <class Vector3s>
    Vector3s rotateInertiaToBase(const Vector3s& vector) const
    {
        if (storedAsEuler())
            return euler_.toRotationMatrix().transpose() * vector;
        else
            return quat_.inverse() * vector;
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
        quat_.setIdentity();
        euler_ = quat_;
        position().setZero();
    }

    void setRandom()
    {
        euler_.setRandom();
        quat_ = euler_.toQuaternion();
        position().setRandom();
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

    QuaternionTpl quat_;
    EulerAnglesXYZTpl euler_;

    Position3Tpl position_;
};

}  // namespace tpl

// convenience typedef (required)
typedef tpl::RigidBodyPose<double> RigidBodyPose;

}  // namespace rbd
}  // namespace ct
