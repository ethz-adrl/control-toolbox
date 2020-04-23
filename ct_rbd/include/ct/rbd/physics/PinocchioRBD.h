
#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wcomment"
#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#pragma GCC diagnostic pop

#include "RigidBodyDynamics.h"
#include "frameHelpers.h"

//#include "TemporaryPinocchioParser.h"

namespace ct {
namespace rbd {

/*!
 * \brief Interface to the pinocchio rbd engine
 * 
 * \warning this is only for fix-base robots.
 * *
 * \note pinocchio cannot differentiate between hard and soft limits.
 * 
 * \note limtis only work for revolute, continuous and prismatic joints.
 */
template <class ROB, typename SCALAR = double>
class PinocchioRBD final : public RigidBodyDynamics<ROB, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = RigidBodyDynamics<ROB, SCALAR>;

    static const size_t NJOINTS = ROB::NJOINTS;

    using JointPosition_t = typename BASE::JointPosition_t;
    using JointVelocity_t = typename BASE::JointVelocity_t;
    using JointAcceleration_t = typename BASE::JointAcceleration_t;
    using JointTorque_t = typename BASE::JointTorque_t;
    using Pose_t = typename BASE::Pose_t;
    using Jacobian_t = typename BASE::Jacobian_t;
    using Twist_t = typename BASE::Twist_t;
    using RotationMatrix_t = typename BASE::RotationMatrix_t;
    using JointSpaceInertiaMatrix_t = typename BASE::JointSpaceInertiaMatrix_t;
    using Wrench_t = typename BASE::Wrench_t;
    using DerivativeMatrix_t = Eigen::Matrix<SCALAR, NJOINTS, NJOINTS>;

    using Model_t = typename pinocchio::template ModelTpl<SCALAR>;
    using Data_t = typename Model_t::Data;
    using JointModel_t = typename pinocchio::template JointModelTpl<SCALAR>;
    using Motion_t = typename pinocchio::template MotionTpl<SCALAR>;
    using Force_t = typename pinocchio::template ForceTpl<SCALAR>;

    using PinocchioForceVector_t = pinocchio::container::aligned_vector<Force_t>;

    PinocchioRBD();

    PinocchioRBD(const PinocchioRBD& other);

    PinocchioRBD* clone() const override;

    bool loadModelFromString(const char* xml_string, const bool verb = false) override;

    bool loadFromURDF(const urdf::ModelInterface* urdf_mdl, const bool verb = false) override;

    bool loadModelFromFile(const std::string& fileName, const bool verb = false) override;

    bool loadModelFromString(const char* xml_string,
        const std::vector<std::string>& move_joints,
        const bool verb = false) override;

    bool loadFromURDF(const urdf::ModelInterface* urdf_mdl,
        const std::vector<std::string>& move_joints,
        const bool verb = false) override;

    bool loadModelFromFile(const std::string& fileName,
        const std::vector<std::string>& move_joints,
        const bool verb = false) override;

    void computeGravityCompensation(const JointPosition_t& p, JointTorque_t& tau) override;

    /*
     * \brief compute the inverse dynamics
     */
    void computeInverseDynamics(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointAcceleration_t& a,
        JointTorque_t& tau) override;

    /*
     * \brief compute the inverse dynamics with external TOOL forces
     * @param p reference joint position
     * @param v reference joint velocity
     * @param a reference joint acceleration
     * @param extToolWrench external wrench applied to the TCP frame in TCP COORDINATES
     * @param tau resulting joint torque
     */
    void computeInverseDynamics(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointAcceleration_t& a,
        const Wrench_t& extWrench_in_tool_frame,
        JointTorque_t& tau);

    /*
     * \brief compute the forward dynamics
     * \warning the current interface of the forward dynamics assumes zero damping.
     */
    void computeForwardDynamics(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointTorque_t& tau,
        JointAcceleration_t& a,
        const bool update_kin = true) override;

    void computeFramePoseInBaseCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Pose_t& result,
        const bool update_kin = true) override;

    void computeBasePoseInFrameCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Pose_t& result,
        const bool update_kin = true) override;

    // compute pose of local frame in world coordinates
    void computeFramePoseInWorldCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Pose_t& result,
        const bool update_kin = true) override;

    // compute pose of world frame in local coordinates
    void computeWorldPoseInFrameCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Pose_t& result,
        const bool update_kin = true) override;

    void computeSpatialJacobianInBaseCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Jacobian_t& jacobian,
        const bool update_kin = true) override;

    void computeSpatialJacobianInWorldCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Jacobian_t& jacobian,
        const bool update_kin = true) override;

    void computeSpatialJacobianInFrameCoordinates(const JointPosition_t& p,
        const typename ROB::Frames frameEnum,
        Jacobian_t& jacobian,
        const bool update_kin = true) override;

    // compute twist in frame coordinates, order = [translational velocity, angular velocity]
    void computeTwistInFrameCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frameEnum,
        Twist_t& twist,
        const bool update_kin = true) override;

    // compute twist in BASE coordinates, order = [translational velocity, angular velocity]
    void computeTwistInBaseCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frameEnum,
        Twist_t& twist,
        const bool update_kin = true) override;

    // compute twist in WORLD coordinates, order = [translational velocity, angular velocity]
    void computeTwistInWorldCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frameEnum,
        Twist_t& twist,
        const bool update_kin = true) override;

    //! perform some elementary checks
    bool checkModel(const Model_t& mdl);

    //! print out info on how the model was parsed
    void printModelInfo(const Model_t& mdl) const;

    //! compute the derivatives of the forward dynamics
    void computeForwardDynamicsDerivatives(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointTorque_t& tau,
        DerivativeMatrix_t& partial_dq,
        DerivativeMatrix_t& partial_dv,
        DerivativeMatrix_t& partial_dtau);

    // compute the inverse of the joint-space inertia matrix (can exploit prior call to computeABADerivatives)
    void computeMinverse(const JointPosition_t& p, JointSpaceInertiaMatrix_t& minv, const bool run_computations = true);

    //! compute the derivatives of the forward dynamics while an external wrench is acting on the TOOL frame
    // todo write documentation for this
    void computeForwardDynamicsDerivatives(const JointPosition_t& p,
        const JointVelocity_t& v,
        const JointTorque_t& tau,
        const Wrench_t& extToolWrench,
        DerivativeMatrix_t& partial_dq,
        DerivativeMatrix_t& partial_dv,
        DerivativeMatrix_t& partial_dtau);

    // compute the jacobian time derivative in local frame
    void computeSpatialJacobianTimeDerivativeInFrameCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frameEnum,
        Jacobian_t& jacTimeDerivative);

    // compute the jacobian time derivative in base coordinates
    void computeSpatialJacobianTimeDerivativeInBaseCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frameEnum,
        Jacobian_t& dJdt);

    // compute the jacobian time derivative in world coordinates
    void computeSpatialJacobianTimeDerivativeInWorldCoordinates(const JointPosition_t& p,
        const JointVelocity_t& v,
        const typename ROB::Frames frameEnum,
        Jacobian_t& dJdt);

private:
    /**
     * @brief maps a wrench formulated in TOOL frame to the frame of the last movable joint.
     * 
     * @param extToolWrench the wrench expressed in TCP coordinate frame
     * @return PinocchioForceVector_t the wrench expressed in the last movable joint
     * 
     * @todo generalize this method for arbitrary joints and frames (not just TCP -> last joint)
     */
    PinocchioForceVector_t projectToolWrenchToLastJoint(const JointPosition_t& p,
        const Wrench_t& extToolWrench,
        const bool update_kin = true);


    //! call this after loading a model
    bool completeModelSetup(const bool verb);

    /*!
     * \brief under the assumption of having a fixed-base robots (which we make in this file), we can save computation time
     *  storing the world-to-base frame pose explicitly.
     */
    bool initializeBaseFrame(const bool verb);

    void assignJointLimits(const Model_t& mdl, const bool verb = false);

    void assignEffortLimits(const Model_t& mdl, const bool verb = false);

    void assignVelocityLimits(const Model_t& mdl, const bool verb = false);

    /**
     * @brief translate a configuration vector in minimal coordinates to a pinocchio-style configuration vector
     * 
     * @param p joint position in minimal coordinates
     * @return full configuration vector including quaternion-style parameterization of continuous joints
     */
    auto toPinocchioConfigVector(const JointPosition_t& p);

    // Pinocchio model instance
    Model_t model_;

    // Pinocchio data instance
    Data_t data_;

    // the pose of the base frame in the world
    Pose_t w_pose_base_;
};

}  // namespace rbd
}  // namespace ct

#include "PinocchioRBD-impl.h"
