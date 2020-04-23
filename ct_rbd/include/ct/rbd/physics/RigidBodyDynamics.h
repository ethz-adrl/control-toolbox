
#pragma once

#include <ct/core/core.h>
#include <ct/rbd/state/RigidBodyPose.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "frameHelpers.h"

namespace ct {
namespace rbd {

template <class ROB, typename SCALAR = double>
class RigidBodyDynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t NJOINTS = ROB::NJOINTS;

    using JointPosition_t = Eigen::Matrix<SCALAR, NJOINTS, 1>;
    using JointVelocity_t = Eigen::Matrix<SCALAR, NJOINTS, 1>;
    using JointAcceleration_t = Eigen::Matrix<SCALAR, NJOINTS, 1>;
    using JointTorque_t = Eigen::Matrix<SCALAR, NJOINTS, 1>;
    using CartesianPosition_t = Eigen::Matrix<SCALAR, 3, 1>;
    using RotationMatrix_t = Eigen::Matrix<SCALAR, 3, 3>;
    using Jacobian_t = Eigen::Matrix<SCALAR, 6, NJOINTS>;
    using Twist_t = Eigen::Matrix<SCALAR, 6, 1>;
    using Wrench_t = Eigen::Matrix<SCALAR, 6, 1>;
    using Pose_t = tpl::RigidBodyPose<SCALAR>;
    using JointSpaceInertiaMatrix_t = Eigen::Matrix<SCALAR, NJOINTS, NJOINTS>;

    RigidBodyDynamics();

    virtual ~RigidBodyDynamics() = default;

    virtual RigidBodyDynamics<ROB, SCALAR>* clone() const = 0;

    virtual bool loadModelFromString(const char* xml_string, const bool verb = false) = 0;

    virtual bool loadModelFromString(const char* xml_string,
        const std::vector<std::string>& movable_joints,
        const bool verb = false) = 0;

    virtual bool loadModelFromFile(const std::string& path, const bool verbose = false) = 0;

    virtual bool loadModelFromFile(const std::string& path,
        const std::vector<std::string>& movable_joints,
        const bool verb = false) = 0;

    virtual bool loadFromURDF(const urdf::ModelInterface* urdf_model, const bool verb = false) = 0;

    virtual bool loadFromURDF(const urdf::ModelInterface* urdf_model,
        const std::vector<std::string>& movable_joints,
        const bool verb = false) = 0;

    virtual void computeForwardDynamics(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointTorque_t& tau,
        JointAcceleration_t& acc,
        const bool update_kin = true) = 0;

    virtual void computeInverseDynamics(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointAcceleration_t& a,
        JointTorque_t& tau) = 0;

    virtual void computeGravityCompensation(const JointPosition_t& p, JointTorque_t& tau) = 0;

    virtual void computeFramePoseInBaseCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Pose_t& pose,
        const bool update_kin = true) = 0;

    virtual void computeBasePoseInFrameCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Pose_t& pose,
        const bool update_kin = true) = 0;

    virtual void computeFramePoseInWorldCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Pose_t& pose,
        const bool update_kin = true) = 0;

    virtual void computeWorldPoseInFrameCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Pose_t& pose,
        const bool update_kin = true) = 0;

    virtual void computeSpatialJacobianInBaseCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Jacobian_t& jac,
        const bool update_kin = true) = 0;

    virtual void computeSpatialJacobianInWorldCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Jacobian_t& jac,
        const bool update_kin = true) = 0;

    virtual void computeSpatialJacobianInFrameCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frame,
        Jacobian_t& jac,
        const bool update_kin = true) = 0;

    virtual void computeTwistInFrameCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frame,
        Twist_t& twist,
        const bool update_kin = true) = 0;

    virtual void computeTwistInBaseCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frame,
        Twist_t& twist,
        const bool update_kin = true) = 0;

    virtual void computeTwistInWorldCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frame,
        Twist_t& twist,
        const bool update_kin = true) = 0;

    void getPositionLimits(JointPosition_t& lowerLim, JointPosition_t& upperLim) const;

    void getEffortLimits(JointTorque_t& effortLim) const;

    void getVelocityLimits(JointVelocity_t& velocityLim) const;


    /*!
     * \brief represent a matrix or vector quantity given in local (frame) coordinates in WORLD coordinates
     */
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime != T::RowsAtCompileTime),
        T>::type  // conversion for non-square matrices
    fromFrameToWorldCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames frame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        // Compute transformation matrix A to rotate into local coordinates
        auto A = computeFromFrameToWorldCoordinatesRotationMatrix(p, frame, update_kin);

        return A * T_in;
    }
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime == T::RowsAtCompileTime), T>::type  // conversion for square matrix
    fromFrameToWorldCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames frame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        // Compute transformation matrix A to rotate into local coordinates
        auto A = computeFromFrameToWorldCoordinatesRotationMatrix(p, frame, update_kin);

        return A * T_in * A.transpose();
    }


    /*!
     * \brief represent a matrix or vector quantity given in local (frame) coordinates in BASE coordinates
     */
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime != T::RowsAtCompileTime),
        T>::type  // conversion for non-square matrices
    fromFrameToBaseCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames frame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        // Compute transformation matrix A to rotate into local coordinates
        auto A = computeFromFrameToBaseCoordinatesRotationMatrix(p, frame, update_kin);

        return A * T_in;
    }
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime == T::RowsAtCompileTime), T>::type  // conversion for square matrix
    fromFrameToBaseCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames frame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        // Compute transformation matrix A to rotate into local coordinates
        auto A = computeFromFrameToBaseCoordinatesRotationMatrix(p, frame, update_kin);

        return A * T_in * A.transpose();
    }


    /*!
     * \brief represent a matrix or vector quantity given in base coordinates in LOCAL (frame) coordinates
     */
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime != T::RowsAtCompileTime),
        T>::type  // conversion for non-square matrices
    fromBaseToFrameCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames frame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        auto A = computeFromBaseToFrameCoordinatesRotationMatrix(p, frame, update_kin);

        return A * T_in;
    }
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime == T::RowsAtCompileTime), T>::type  // conversion for square matrix
    fromBaseToFrameCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames frame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        auto A = computeFromBaseToFrameCoordinatesRotationMatrix(p, frame, update_kin);

        return A * T_in * A.transpose();
    }


    /*!
     * \brief represent a matrix or vector quantity given in Frame A coordinates in Frame B
     * @param T_in the screw-type quantity to be transformed
     * @param p the current joint position
     * @param fromFrame frame in which T_in is represented now
     * @param toFrame frame to which T_in is to be transformed
     * @param update_kin update kinematics (yes/no)
     */
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime != T::RowsAtCompileTime),
        T>::type  // conversion for non-square matrices
    fromFrameToFrameCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames fromFrame,
        const typename ROB::Frames toFrame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        // Compute transformation matrix A to rotate into local coordinates
        auto A = computeFromFrameToFrameCoordinatesRotationMatrix(p, fromFrame, toFrame, update_kin);

        return A * T_in;
    }
    template <typename T>
    typename std::enable_if<(T::ColsAtCompileTime == T::RowsAtCompileTime), T>::type  // conversion for square matrix
    fromFrameToFrameCoordinates(const T& T_in,
        const JointPosition_t& p,
        const typename ROB::Frames fromFrame,
        const typename ROB::Frames toFrame,
        const bool update_kin = true)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(T);
        // Compute transformation matrix A to rotate into local coordinates
        auto A = computeFromFrameToFrameCoordinatesRotationMatrix(p, fromFrame, toFrame, update_kin);

        return A * T_in * A.transpose();
    }


protected:
    //! compute the transformation matrix for rotation between different frames
    Eigen::Matrix<SCALAR, 6, 6> computeFromFrameToFrameCoordinatesRotationMatrix(const JointPosition_t& q,
        const typename ROB::Frames fromFrame,
        const typename ROB::Frames toFrame,
        const bool update_kin = true);

    //! compute the transformation matrix for rotation from a specific frame to the world frame
    Eigen::Matrix<SCALAR, 6, 6> computeFromFrameToWorldCoordinatesRotationMatrix(const JointPosition_t& q,
        const typename ROB::Frames frame,
        const bool update_kin = true);

    //! compute the transformation matrix for rotation from a specific frame to the base frame
    Eigen::Matrix<SCALAR, 6, 6> computeFromFrameToBaseCoordinatesRotationMatrix(const JointPosition_t& q,
        const typename ROB::Frames frame,
        const bool update_kin = true);

    //! compute the transformation matrix for rotation from a base frame to a specific frame
    Eigen::Matrix<SCALAR, 6, 6> computeFromBaseToFrameCoordinatesRotationMatrix(const JointPosition_t& q,
        const typename ROB::Frames frame,
        const bool update_kin = true);

    // hard joint limits
    JointPosition_t p_lim_lower_;
    JointPosition_t p_lim_upper_;

    // torque limit (positive)
    JointTorque_t tau_lim_;

    // velocity limit (positive)
    JointVelocity_t v_lim_;
};


}  // namespace rbd
}  // namespace ct

#include "RigidBodyDynamics-impl.h"