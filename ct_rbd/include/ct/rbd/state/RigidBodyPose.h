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

#pragma once

#include <kindr/Core>

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

    enum STORAGE_TYPE
    {
        QUAT = 0,
        EULER = 1
    };

    typedef kindr::HomogeneousTransformationPosition3RotationQuaternion<SCALAR> HomogeneousTransform;
    typedef kindr::Position<SCALAR, 3> Position3Tpl;
    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3Tpl;


    RigidBodyPose(STORAGE_TYPE storage = EULER)
        : storage_(storage),
          quat_(SCALAR(1.0), SCALAR(0.0), SCALAR(0.0), SCALAR(0.0)),  // for CppAD cg compatibility
          euler_(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0))
    {
    }

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

    RigidBodyPose(const RigidBodyPose<SCALAR>& arg):
    	storage_(arg.storage_),
		quat_(arg.quat_),
		euler_(arg.euler_),
		position_(arg.position_)
    {}

    //RigidBodyPose(const Eigen::Vector3d& orientationEulerXyz, const Eigen::Vector3d& position, STORAGE_TYPE storage = QUAT);
    //RigidBodyPose(const Eigen::Quaterniond& orientationQuat, const Eigen::Vector3d& position, STORAGE_TYPE storage = QUAT);

    ~RigidBodyPose(){};

//! @todo why do we need this operator overloaded? It obviously skips a few important members
//    inline void operator=(const RigidBodyPose& rhs)
//    {
//        setFromRotationQuaternion(rhs.getRotationQuaternion());
//        position() = rhs.position();
//    }

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
            return kindr::RotationMatrix<SCALAR>(euler_);
        }
        else
        {
            return kindr::RotationMatrix<SCALAR>(quat_);
        }
    }

    /**
	 * \brief This method sets the Euler angles rotation (X,Y,Z / roll,pitch,yaw) from a kinder type.
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
	 * \brief This method sets the Euler angles rotation (X,Y,Z / roll,pitch,yaw) from a kinder type.
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
	 * \brief This method sets the quaternion angles rotation (X,Y,Z / roll,pitch,yaw) from a kinder type.
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
	 * \brief This method sets the quaternion angles rotation (X,Y,Z / roll,pitch,yaw) from a kinder type.
	 */
    void setFromRotationQuaternion(const Eigen::Quaterniond& quat)
    {
        if (storedAsEuler())
        {
            euler_ = kindr::RotationQuaternion<SCALAR>(quat);
        }
        else
        {
            quat_ = kindr::RotationQuaternion<SCALAR>(quat);
        }
    }

    /**
	 * \brief This method returns the position of the Base frame in the inertia frame.
	 */
    const Position3Tpl& position() const { return position_; };
    /**
	 * \brief This method returns the position of the Base frame in the inertia frame.
	 */
    Position3Tpl& position() { return position_; };
    /**
	 * \brief This methods rotates a 3D vector expressed in Base frame to Inertia Frame.
	 */
    template <class Vector3s>
    Vector3s rotateBaseToInertia(const Vector3s& vector) const
    {
        if (storedAsEuler())
        {
            Eigen::AngleAxis<SCALAR> rollAngle(euler_.toImplementation()(0), Eigen::Matrix<SCALAR, 3, 1>::UnitX());
            Eigen::AngleAxis<SCALAR> pitchAngle(euler_.toImplementation()(1), Eigen::Matrix<SCALAR, 3, 1>::UnitY());
            Eigen::AngleAxis<SCALAR> yawAngle(euler_.toImplementation()(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ());
            Eigen::Quaternion<SCALAR> q = rollAngle * pitchAngle * yawAngle;
            Eigen::Matrix<SCALAR, 3, 3> rotationMatrix = q.matrix();

            // incredibly ungly hack to get along with different types
            Eigen::Matrix<SCALAR, 3, 1> vec_temp;
            vec_temp << vector(0), vector(1), vector(2);

            Eigen::Matrix<SCALAR, 3, 1> result = rotationMatrix * vec_temp;
            return (Vector3s)result;

            //            return euler_.rotate(vector); // temporarily replaced -- the kindr rotate() method is not auto-diffable
        }
        else
        {
            return quat_.rotate(vector);
        }
    };


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
    };

    /**
	 * \brief This methods returns the Homogeneous transform from the Base frame to the inertia frame.
	 */
    HomogeneousTransform getHomogeneousTransform() const
    {
        throw std::runtime_error("get homogeneous transform not implemented");
        return HomogeneousTransform();
    };

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

    //! would we have a non-const accessor here?
//    bool storedAsEuler()
//    {
//        if (storage_ == EULER)
//            return true;
//        else
//            return false;
//    }

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

typedef tpl::RigidBodyPose<double> RigidBodyPose;


}  // namespace rbd
}  // namespace ct
