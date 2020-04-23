
#pragma once

namespace ct {
namespace rbd {

template <class ROB, typename SCALAR>
RigidBodyDynamics<ROB, SCALAR>::RigidBodyDynamics()
{
    /*
     * Choose the absolute highest value for the bounds
     * that remains being meaningful for the SCALAR type
     * in use.
     */
    SCALAR lower = std::numeric_limits<SCALAR>::lowest();
    SCALAR upper = std::numeric_limits<SCALAR>::max();
    p_lim_lower_ = lower * JointPosition_t::Ones();
    p_lim_upper_ = upper * JointPosition_t::Ones();
    tau_lim_ = upper * JointTorque_t::Ones();
    v_lim_ = upper * JointVelocity_t::Ones();
}

template <class ROB, typename SCALAR>
void RigidBodyDynamics<ROB, SCALAR>::getPositionLimits(JointPosition_t& lowerLim, JointPosition_t& upperLim) const
{
    lowerLim = p_lim_lower_;
    upperLim = p_lim_upper_;
}

template <class ROB, typename SCALAR>
void RigidBodyDynamics<ROB, SCALAR>::getEffortLimits(JointTorque_t& effortLim) const
{
    effortLim = tau_lim_;
}

template <class ROB, typename SCALAR>
void RigidBodyDynamics<ROB, SCALAR>::getVelocityLimits(JointVelocity_t& velocityLim) const
{
    velocityLim = v_lim_;
}


template <class ROB, typename SCALAR>
Eigen::Matrix<SCALAR, 6, 6> RigidBodyDynamics<ROB, SCALAR>::computeFromFrameToFrameCoordinatesRotationMatrix(
    const JointPosition_t& p,
    const typename ROB::Frames fromFrame,
    const typename ROB::Frames toFrame,
    const bool update_kinematics)
{
    if (fromFrame == toFrame)  // catch corner case for efficiency
    {
        return Eigen::Matrix<SCALAR, 6, 6>::Identity();
    }

    // Compute transformation matrix A to rotate into local coordinates
    Eigen::Matrix<SCALAR, 6, 6> A = Eigen::Matrix<SCALAR, 6, 6>::Zero();
    Pose_t b_pose_f_in, b_pose_f_out;

    if (fromFrame != ROB::BASE)  // check edge case to save computation time
        computeFramePoseInBaseCoordinates(p, fromFrame, b_pose_f_in, update_kinematics);
    else
        b_pose_f_in.setIdentity();  // base-to-base pose is identity

    if (toFrame != ROB::BASE)
        computeBasePoseInFrameCoordinates(p, toFrame, b_pose_f_out, update_kinematics);
    else
    {
        b_pose_f_out.setIdentity();
    }

    RotationMatrix_t R1 = b_pose_f_in.R().toImplementation();
    RotationMatrix_t R2 = b_pose_f_out.R().toImplementation();

    A.template block<3, 3>(0, 0) = R2 * R1;
    A.template block<3, 3>(3, 3) = R2 * R1;

    return A;
}

template <class ROB, typename SCALAR>
Eigen::Matrix<SCALAR, 6, 6> RigidBodyDynamics<ROB, SCALAR>::computeFromFrameToWorldCoordinatesRotationMatrix(
    const JointPosition_t& p,
    const typename ROB::Frames frameEnum,
    const bool update_kinematics)
{
    // Compute transformation matrix A to rotate into local coordinates
    Eigen::Matrix<SCALAR, 6, 6> A = Eigen::Matrix<SCALAR, 6, 6>::Zero();
    Pose_t b_pose_f;
    computeFramePoseInWorldCoordinates(p, frameEnum, b_pose_f, update_kinematics);
    RotationMatrix_t R = b_pose_f.R().toImplementation();
    A.template block<3, 3>(0, 0) = R;
    A.template block<3, 3>(3, 3) = R;
    return A;
}


template <class ROB, typename SCALAR>
Eigen::Matrix<SCALAR, 6, 6> RigidBodyDynamics<ROB, SCALAR>::computeFromFrameToBaseCoordinatesRotationMatrix(
    const JointPosition_t& p,
    const typename ROB::Frames frameEnum,
    const bool update_kinematics)
{
    if (frameEnum == ROB::Frames::BASE)  // catch corner case for efficiency
    {
        return Eigen::Matrix<SCALAR, 6, 6>::Identity();
    }

    // Compute transformation matrix A to rotate into local coordinates
    Eigen::Matrix<SCALAR, 6, 6> A = Eigen::Matrix<SCALAR, 6, 6>::Zero();
    Pose_t b_pose_f;
    computeFramePoseInBaseCoordinates(p, frameEnum, b_pose_f, update_kinematics);
    RotationMatrix_t R = b_pose_f.R().toImplementation();
    A.template block<3, 3>(0, 0) = R;
    A.template block<3, 3>(3, 3) = R;
    return A;
}


template <class ROB, typename SCALAR>
Eigen::Matrix<SCALAR, 6, 6> RigidBodyDynamics<ROB, SCALAR>::computeFromBaseToFrameCoordinatesRotationMatrix(
    const JointPosition_t& p,
    const typename ROB::Frames frameEnum,
    const bool update_kinematics)
{
    if (frameEnum == ROB::Frames::BASE)  // catch corner case for efficiency
    {
        return Eigen::Matrix<SCALAR, 6, 6>::Identity();
    }

    // Compute transformation matrix A to rotate into local coordinates
    Eigen::Matrix<SCALAR, 6, 6> A = Eigen::Matrix<SCALAR, 6, 6>::Zero();
    Pose_t b_pose_f;
    computeBasePoseInFrameCoordinates(p, frameEnum, b_pose_f, update_kinematics);
    RotationMatrix_t R = b_pose_f.R().toImplementation();
    A.template block<3, 3>(0, 0) = R;
    A.template block<3, 3>(3, 3) = R;
    return A;
}

}  // namespace rbd
}  // namespace ct
