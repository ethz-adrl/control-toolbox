/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"
#include <kindr/Core>
#pragma GCC diagnostic pop

#include <ct/rbd/state/JointState.h>

namespace ct {
namespace rbd {

/**
 * \brief Container class containing all robcogen classes
 */
template <class RBDTrait, template <typename> class LinkDataMapT, class U>
class RobCoGenContainer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobCoGenContainer()
        : homogeneousTransforms_(),
          motionTransforms_(),
          forceTransforms_(),
          jacobians_(),
          inertiaProperties_(),
          jSim_(inertiaProperties_, forceTransforms_),
          forwardDynamics_(inertiaProperties_, motionTransforms_),
          inverseDynamics_(inertiaProperties_, motionTransforms_){};

    typedef typename RBDTrait::S SCALAR;

    typedef RBDTrait TRAIT;

    typedef U UTILS;

    static const size_t NJOINTS = RBDTrait::joints_count;
    static const size_t NLINKS = RBDTrait::links_count;

    typedef RobCoGenContainer<RBDTrait, LinkDataMapT, UTILS> specialized_t;
    typedef std::shared_ptr<specialized_t> Ptr_t;

    typedef typename RBDTrait::HomogeneousTransforms HomogeneousTransforms;
    typedef typename RBDTrait::MotionTransforms MotionTransforms;
    typedef typename RBDTrait::ForceTransforms ForceTransforms;
    typedef typename RBDTrait::Jacobians Jacobians;
    typedef typename RBDTrait::InertiaProperties InertiaProperties;
    typedef typename RBDTrait::JSIM JSIM;
    typedef typename RBDTrait::FwdDynEngine ForwardDynamics;
    typedef typename RBDTrait::InvDynEngine InverseDynamics;
    typedef typename RBDTrait::LinkID LinkIdentifiers;

    typedef LinkDataMapT<Eigen::Matrix<SCALAR, 6, 1>> LinkForceMap;

    typedef Eigen::Matrix<SCALAR, 4, 4> HomogeneousTransform;
    typedef Eigen::Matrix<SCALAR, 6, 6> ForceTransform;
    typedef Eigen::Matrix<SCALAR, 6, NJOINTS> Jacobian;
    typedef Eigen::Matrix<SCALAR, 3, 3> Matrix3Tpl;

    typedef kindr::Position<SCALAR, 3> Position3Tpl;
    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3Tpl;

    using RigidBodyPoseTpl = tpl::RigidBodyPose<SCALAR>;


    HomogeneousTransforms& homogeneousTransforms() { return homogeneousTransforms_; };
    const HomogeneousTransforms& homogeneousTransforms() const { return homogeneousTransforms_; };
    MotionTransforms& motionTransforms() { return motionTransforms_; };
    const MotionTransforms& motionTransforms() const { return motionTransforms_; };
    ForceTransforms& forceTransforms() { return forceTransforms_; };
    const ForceTransforms& forceTransforms() const { return forceTransforms_; };
    Jacobians& jacobians() { return jacobians_; };
    const Jacobians& jacobians() const { return jacobians_; };
    InertiaProperties& inertiaProperties() { return inertiaProperties_; };
    const InertiaProperties& inertiaProperties() const { return inertiaProperties_; };
    JSIM& jSim() { return jSim_; }
    const JSIM& jSim() const { return jSim_; }
    ForwardDynamics& forwardDynamics() { return forwardDynamics_; };
    const ForwardDynamics& forwardDynamics() const { return forwardDynamics_; };
    InverseDynamics& inverseDynamics() { return inverseDynamics_; };
    const InverseDynamics& inverseDynamics() const { return inverseDynamics_; };
    /**
	 * \brief Get a homogeneous transformation from link to base provided a link id
	 *
	 * The homogeneous transform takes a quantity expressed in the link frame and transforms
	 * it into the base frame, i.e. \f$ {_Bx} = T_{BL} {_Lx} \f$
	 * where \f$ T_{BL}  \f$ is the transform you obtain from this function.
	 *
	 * @param linkId link id
	 * @param position joint positions (angles)
	 * @return Homogeneous transformation from link to base \f$ T_{BL}  \f$
	 */
    HomogeneousTransform getHomogeneousTransformBaseLinkById(size_t linkId,
        const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        return UTILS::getTransformBaseLinkById(homogeneousTransforms(), linkId, jointPosition);
    }

    /**
	 * \brief Get a force transformation from link to base provided a link id
	 *
	 * The force transform takes a torque-force vector expressed in the base frame and transforms
	 * it into the link frame, i.e. \f$ {_Lx} = T_{LB} {_Bx} \f$
	 * where \f$ T_{LB}  \f$ is the transform you obtain from this function.
	 *
	 * @param linkId link id
	 * @param position joint positions (angles)
	 * @return Force transformation from link to base \f$ T_{LB}  \f$
	 */
    ForceTransform getForceTransformLinkBaseById(size_t linkId,
        const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        return UTILS::getTransformLinkBaseById(forceTransforms(), linkId, jointPosition);
    }

    /**
	 * \brief Get a homogeneous transformation from base to an endeffector provided an endeffector id
	 *
	 * The homogeneous transform converts a quantity expressed in the base and converts it to the end-effector frame, i.e.
	 * \f$ {_EEx} = T_{EE-B} {_Bx}  \f$
	 * The endeffector frame corresponds to the convention used when creating the RobCoGen code.
	 *
	 * @param eeId End-effector ID
	 * @param jointPosition Joint angles/positions
	 * @return Homogeneous transformation from base to endeffector \f$ T_{EE-B}  \f$
	 */
    HomogeneousTransform getHomogeneousTransformBaseEEById(size_t eeId,
        const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        return UTILS::getTransformBaseEEById(homogeneousTransforms(), eeId, jointPosition);
    }


    /*!
	 * \brief Get the end-effector Jacobian expressed in the base frame
	 * @param eeId endeffector ID
	 * @param jointPosition current joint position
	 * @return Jacobian of the endeffector expressed in the base frame
	 */
    Jacobian getJacobianBaseEEbyId(size_t eeId, const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        return UTILS::getJacobianBaseEEbyId(jacobians(), eeId, jointPosition);
    }

    //	/**
    //	 * \brief Get a force transformation from base to an endeffector provided an endeffector id
    //	 *
    //	 * The force transform converts a torque-force vector expressed in the base and converts it to the end-effector frame, i.e.
    //	 * \f$ {_EEx} = T_{EE-B} {_Bx}  \f$
    //	 * The endeffector frame corresponds to the convention used when creating the RobCoGen code.
    //	 *
    //	 * @param eeId End-effector ID
    //	 * @param jointPosition Joint angles/positions
    //	 * @return Homogeneous transformation from base to endeffector \f$ T_{EE-B}  \f$
    //	 */
    // ForceTransform getForceTransformEEBaseById(size_t eeId, const typename JointState<NJOINTS>::Position& jointPosition);

    /*!
	 * \brief Get the endeffector position expressed in the base frame
	 *
	 * @param eeId endeffector ID
	 * @param jointPosition current joint position
	 * @return position of the endeffector expressed in the base frame
	 */
    Position3Tpl getEEPositionInBase(size_t eeId, const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        return Position3Tpl(getHomogeneousTransformBaseEEById(eeId, jointPosition).template topRightCorner<3, 1>());
    }

    /*!
     * \brief Get the endeffector pose expressed in the base frame
     *
	 * @param eeId endeffector ID
	 * @param jointPosition current joint position
	 * @param storage the type of storage inteded for the pose
	 * @return position of the endeffector expressed in the base frame
     */
    RigidBodyPoseTpl getEEPoseInBase(size_t eeId,
        const typename JointState<NJOINTS, SCALAR>::Position& jointPosition,
        typename RigidBodyPoseTpl::STORAGE_TYPE storage = RigidBodyPoseTpl::EULER)
    {
        // construct the rigid body pose from a homogeneous transformation matrix
        RigidBodyPoseTpl pose(getHomogeneousTransformBaseEEById(eeId, jointPosition), storage);
        return pose;
    }

    /*!
     * compute the forward kinematics and return a rotation matrix specifying the ee-rotation w.r.t. the base frame
     */
    Matrix3Tpl getEERotInBase(size_t eeId, const typename JointState<NJOINTS, SCALAR>::Position& jointPosition)
    {
        return getHomogeneousTransformBaseEEById(eeId, jointPosition).template topLeftCorner<3, 3>();
    }


private:
    HomogeneousTransforms homogeneousTransforms_;
    MotionTransforms motionTransforms_;
    ForceTransforms forceTransforms_;
    Jacobians jacobians_;
    InertiaProperties inertiaProperties_;
    JSIM jSim_;
    ForwardDynamics forwardDynamics_;
    InverseDynamics inverseDynamics_;
};


}  // namespace rbd
}  // namespace ct
