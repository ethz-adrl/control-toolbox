/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <unordered_map>

#include <ct/rbd/common/SpatialForceVector.h>
#include <ct/rbd/state/RBDState.h>

#include "kinematics/EndEffector.h"
#include "kinematics/FloatingBaseTransforms.h"
#include "kinematics/InverseKinematicsBase.h"

namespace ct {
namespace rbd {

/**
 * \brief A general class for computing Kinematic properties
 *
 * This class implements useful Kinematic quantities. It wraps RobCoGen to
 * have access to efficient transforms and jacobians.
 */
template <class RBD, size_t N_EE>
class Kinematics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Kinematics(std::shared_ptr<RBD> rbdContainer = std::shared_ptr<RBD>(new RBD()))
        : rbdContainer_(rbdContainer), floatingBaseTransforms_(rbdContainer_)
    {
        initEndeffectors(endEffectors_);
    }

    Kinematics(const Kinematics<RBD, N_EE>& other)
        : rbdContainer_(new RBD()), endEffectors_(other.endEffectors_), floatingBaseTransforms_(rbdContainer_)
    {
    }

    virtual ~Kinematics() = default;

    Kinematics<RBD, N_EE>* clone() const { return new Kinematics<RBD, N_EE>(*this); }
    static const size_t NUM_EE = N_EE;
    static const size_t NJOINTS = RBD::NJOINTS;
    static const size_t NLINKS = RBD::NLINKS;

    using Ptr_t = std::shared_ptr<Kinematics<RBD, N_EE>>;

    using ROBCOGEN = RBD;
    using SCALAR = typename ROBCOGEN::SCALAR;

    using HomogeneousTransform = typename ROBCOGEN::HomogeneousTransform;
    using HomogeneousTransforms = typename ROBCOGEN::HomogeneousTransforms;
    using ForceTransform = typename ROBCOGEN::ForceTransform;
    using Jacobian = typename ROBCOGEN::Jacobian;
    using Jacobians = typename ROBCOGEN::Jacobians;
    using Vector3Tpl = Eigen::Matrix<SCALAR, 3, 1>;
    using Matrix3Tpl = Eigen::Matrix<SCALAR, 3, 3>;
    using Position3Tpl = kindr::Position<SCALAR, 3>;
    using Velocity3Tpl = kindr::Velocity<SCALAR, 3>;
    using QuaterionTpl = kindr::RotationQuaternion<SCALAR>;
    using RigidBodyPoseTpl = tpl::RigidBodyPose<SCALAR>;
    using EEForce = SpatialForceVector<SCALAR>;
    using EEForceLinear = Vector3Tpl;
    using JointState_t = JointState<NJOINTS, SCALAR>;

    void initEndeffectors(std::array<EndEffector<NJOINTS, SCALAR>, NUM_EE>& endeffectors)
    {
        for (size_t i = 0; i < NUM_EE; i++)
        {
            endeffectors[i].setLinkId(ROBCOGEN::UTILS::eeIdToLinkId(i));
        }
    }

    /**
     * \brief Get an end-effector
     * @param id end-effector id
     * @return
     */
    EndEffector<NJOINTS, SCALAR>& getEndEffector(size_t id) { return endEffectors_[id]; }
    /**
     * \brief Set an end-effector
     * @param id end-effector id
     * @param ee end-effector
     */
    void setEndEffector(size_t id, const EndEffector<NJOINTS, SCALAR>& ee){};

    Jacobian getJacobianBaseEEbyId(size_t eeId, const RBDState<NJOINTS, SCALAR>& rbdState)
    {
        return robcogen().getJacobianBaseEEbyId(eeId, rbdState.jointPositions());
    }

    FloatingBaseTransforms<RBD>& floatingBaseTransforms()
    {
        throw std::runtime_error("floating base transforms not implemented");
        return floatingBaseTransforms_;
    }

    FloatingBaseTransforms<RBD>& floatingBaseTransforms() const
    {
        throw std::runtime_error("floating base transforms not implemented");
        return floatingBaseTransforms_;
    }

    const HomogeneousTransforms& transforms() const { return robcogen().homogeneousTransforms(); }
    HomogeneousTransforms& transforms() { return robcogen().homogeneousTransforms(); }
    const Jacobians& jacobians() const { return robcogen().jacobians(); }
    Jacobians& jacobians() { return robcogen().jacobians(); }
    Velocity3Tpl getEEVelocityInBase(size_t eeId, const RBDState<NJOINTS, SCALAR>& rbdState)
    {
        Velocity3Tpl eeVelocityBase;
        eeVelocityBase.toImplementation() =
            (robcogen().getJacobianBaseEEbyId(eeId, rbdState.jointPositions()) * rbdState.jointVelocities())
                .template bottomRows<3>();

        // add translational velocity induced by linear base motion
        eeVelocityBase += rbdState.base().velocities().getTranslationalVelocity();

        // add translational velocity induced by angular base motion
        eeVelocityBase += rbdState.base().velocities().getRotationalVelocity().cross(
            getEEPositionInBase(eeId, rbdState.jointPositions()));

        return eeVelocityBase;
    }

    Velocity3Tpl getEEVelocityInWorld(size_t eeId, const RBDState<NJOINTS, SCALAR>& rbdState)
    {
        Velocity3Tpl eeVelocityBase = getEEVelocityInBase(eeId, rbdState);
        return rbdState.base().pose().rotateBaseToInertia(eeVelocityBase);
    }

    /*!
     * Computes the forward kinematics for the end-effector position and expresses the end-effector position in robot
     * base coordinates.
     * @param eeID unique identifier of the end-effector in question
     * @param jointPosition current robot joint positions
     * @return the current end-effector position in base coordinates
     *
     *      * @todo integrate this into getEEPoseInBase
     *
     */
    Position3Tpl getEEPositionInBase(size_t eeID, const typename JointState_t::Position& jointPosition)
    {
        return robcogen().getEEPositionInBase(eeID, jointPosition);
    }

    /*!
     * Computes the forward kinematics for the end-effector position and expresses the end-effector pose in robot base
     * coordinates.
     * @param eeID unique identifier of the end-effector in question
     * @param jointPosition current robot joint positions
     * @return the current end-effector pose in base coordinates
     */
    RigidBodyPoseTpl getEEPoseInBase(size_t eeID, const typename JointState_t::Position& jointPosition)
    {
        return robcogen().getEEPoseInBase(eeID, jointPosition);
    }

    /*!
     * compute the forward kinematics and return a rotation matrix specifying the ee-rotation w.r.t. the base frame
     */
    Matrix3Tpl getEERotInBase(size_t eeID, const typename JointState_t::Position& jointPosition)
    {
        return robcogen().getEERotInBase(eeID, jointPosition);
    }

    /*!
     * Computes the forward kinematics for the end-effector position and expresses the end-effector position in world
     * coordinates
     * @param eeID unique identifier of the end-effector in question
     * @param basePose current robot base pose
     * @param jointPosition current robot joint positions
     * @return the current end-effector position in world coordinates
     *
     * @todo integrate this into getEEPoseInWorld
     */
    Position3Tpl getEEPositionInWorld(size_t eeID,
        const RigidBodyPoseTpl& basePose,
        const typename JointState_t::Position& jointPosition)
    {
        // vector from base to endeffector expressed in base frame
        Position3Tpl B_x_EE = getEEPositionInBase(eeID, jointPosition);

        // vector from base to endeffector expressed in world frame
        Position3Tpl W_x_EE = basePose.template rotateBaseToInertia(B_x_EE);

        // vector from origin to endeffector = vector from origin to base + vector from base to endeffector
        return basePose.position() + W_x_EE;
    }

    //! get the end-effector pose in world coordinates
    RigidBodyPoseTpl getEEPoseInWorld(size_t eeID,
        const RigidBodyPoseTpl& basePose,
        const typename JointState_t::Position& jointPosition)
    {
        // ee pose in base coordinates
        RigidBodyPoseTpl B_x_EE = getEEPoseInBase(eeID, jointPosition);

        // position rotated into world frame
        Position3Tpl W_p_EE = basePose.template rotateBaseToInertia(B_x_EE.position());

        // orientation rotated into world frame
        QuaterionTpl B_q_EE = B_x_EE.getRotationQuaternion();
        QuaterionTpl W_q_EE = basePose.template rotateBaseToInertiaQuaternion(B_q_EE);

        return RigidBodyPoseTpl(W_q_EE, basePose.position() + W_p_EE);
    }

    //! get the end-effector rotation matrix expressed in world coordinates
    Matrix3Tpl getEERotInWorld(size_t eeID,
        const RigidBodyPoseTpl& basePose,
        const typename JointState_t::Position& jointPosition)
    {
        // ee rotation matrix in base coordinates
        Matrix3Tpl B_R_EE = getEERotInBase(eeID, jointPosition);

        // ee rotation matriix in world
        return basePose.template rotateBaseToInertiaMat(B_R_EE);
    }

    void addIKSolver(const std::shared_ptr<InverseKinematicsBase<NJOINTS, SCALAR>>& solver,
        size_t eeID,
        size_t solverID = 0)
    {
        if (solverID >= 100)
            throw std::runtime_error("Solver ID must be less than 100.");

        size_t hash = eeID * 100 + solverID;
        if (ikSolvers_.find(hash) != ikSolvers_.end())
            throw std::runtime_error("Solver with the same eeID and solverID already present.");
        ikSolvers_[hash] = solver;
    }

    std::shared_ptr<InverseKinematicsBase<NJOINTS, SCALAR>> getIKSolver(const size_t eeID,
        const size_t solverID = 0) const
    {
        if (solverID >= 100)
            throw std::runtime_error("Solver ID must be less than 100.");
        size_t hash = eeID * 100 + solverID;
        return ikSolvers_[hash];
    }

    /**
     * \brief Transforms a force applied at an end-effector and expressed in the world into the link frame the EE is
     * rigidly connected to.
     * @param W_force Force expressed in world coordinates
     * @param basePose Pose of the base (in the world)
     * @param jointPosition Joint angles
     * @param eeId ID of the end-effector
     * @return
     */
    EEForce mapForceFromWorldToLink3d(const Vector3Tpl& W_force,
        const RigidBodyPoseTpl& basePose,
        const typename JointState_t::Position& jointPosition,
        size_t eeId)
    {
        EEForce eeForce(EEForce::Zero());
        eeForce.force() = W_force;

        return mapForceFromWorldToLink(eeForce, basePose, jointPosition, eeId);
    }

    /**
     * \brief Transforms a force applied at an end-effector and expressed in the world into the link frame the EE is
     * rigidly connected to.
     * @param W_force Force expressed in world coordinates
     * @param basePose Pose of the base (in the world)
     * @param jointPosition Joint angles
     * @param eeId ID of the end-effector
     * @return
     */
    EEForce mapForceFromWorldToLink(const EEForce& W_force,
        const RigidBodyPoseTpl& basePose,
        const typename JointState_t::Position& jointPosition,
        size_t eeId)
    {
        // get endeffector position in world
        Position3Tpl B_x_EE = getEEPositionInBase(eeId, jointPosition);

        return mapForceFromWorldToLink(W_force, basePose, jointPosition, B_x_EE, eeId);
    }

    /**
     * \brief Transforms a force applied at an end-effector and expressed in the world into the link frame the EE is
     * rigidly connected to.
     * @param W_force Force expressed in world coordinates
     * @param basePose Pose of the base (in the world)
     * @param jointPosition Joint angles
     * @param B_x_EE Position of the end effector in the base
     * @param eeId ID of the end-effector
     * @return
     */
    EEForce mapForceFromWorldToLink(const EEForce& W_force,
        const RigidBodyPoseTpl& basePose,
        const typename JointState_t::Position& jointPosition,
        const Position3Tpl& B_x_EE,
        size_t eeId)
    {
        // get the link that the EE is attached to
        size_t linkId = getEndEffector(eeId).getLinkId();

        // transform the force/torque to an equivalent force/torque in the base using a lever-arm for the torque
        EEForce B_force;

        B_force.force() = basePose.template rotateInertiaToBase<Vector3Tpl>(W_force.force());
        B_force.torque() = B_x_EE.toImplementation().cross(B_force.force()) +
                           basePose.template rotateInertiaToBase<Vector3Tpl>(W_force.torque());

        // transform force to link on which endeffector sits on
        return EEForce(robcogen().getForceTransformLinkBaseById(linkId, jointPosition) * B_force);
    }

    /**
     * \brief Transforms a force applied at an end-effector expressed in an arbitrary (end-effector)
     * frame into the link frame the EE is rigidly connected to.
     *
     * This function does not assume any position or orientation of the end-effector. Therefore,
     * the user must specify the orientation of the end-effector in the base. There are two common
     * choices: use zero rotation to assume forces/torques are expressed in the base. Or use the
     * base orientation to assume the end-effector forces/torques are expressed in the world. For
     * this variant, also see mapForceFromWorldToLink().
     *
     * NOTE: Even if zero or base rotation is assumed, you have to pass the correct position of the
     * end-effector expressed in the base as part of T_B_EE! Do NOT pass the base pose directly here!
     *
     * @param EE_force 6D torque/force vector expressed in the EE frame
     * @param T_B_EE transform from end-effector to base
     * @param jointPosition joint angles
     * @param eeId ID of the end-effector
     * @return
     */
    EEForce mapForceFromEEToLink(const EEForce& EE_force,
        const RigidBodyPoseTpl& T_B_EE,
        const typename JointState_t::Position& jointPosition,
        size_t eeId)
    {
        // get the link that the EE is attached to
        size_t linkId = getEndEffector(eeId).getLinkId();

        // transform the force/torque to an equivalent force/torque in the base using a lever-arm for the torque
        EEForce B_force;

        // convenience accessors
        Vector3Tpl B_x_EE = T_B_EE.position().toImplementation();

        // rotate from the end-effector frame to the inertia frame
        B_force.force() = T_B_EE.template rotateBaseToInertia<Vector3Tpl>(EE_force.force());

        // build the cross product and add the torque (which first is rotated from the end-effector to the base)
        B_force.torque() =
            B_x_EE.cross(B_force.force()) + T_B_EE.template rotateBaseToInertia<Vector3Tpl>(EE_force.torque());

        // transform force to link on which endeffector sits on
        return EEForce(robcogen().getForceTransformLinkBaseById(linkId, jointPosition) * B_force);
    };

    RBD& robcogen() { return *rbdContainer_; }
private:
    std::shared_ptr<RBD> rbdContainer_;
    std::array<EndEffector<NJOINTS, SCALAR>, N_EE> endEffectors_;
    FloatingBaseTransforms<RBD> floatingBaseTransforms_;

    std::unordered_map<size_t, std::shared_ptr<InverseKinematicsBase<NJOINTS, SCALAR>>> ikSolvers_;
};

} /* namespace rbd */
} /* namespace ct */
