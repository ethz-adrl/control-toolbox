/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/optcon.h>

#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace rbd {

/*!
 * \brief A costfunction term that defines a cost on a task-space pose, for fix-base robots only
 *
 * \tparam KINEMATICS kinematics of the system
 * \tparam STATE_DIM state dimensionality of the system
 * \tparam CONTROL_DIM control dimensionality of the system
 */
template <class KINEMATICS, size_t STATE_DIM, size_t CONTROL_DIM>
class TermTaskspaceGeometricJacobian : public optcon::TermBase<STATE_DIM, CONTROL_DIM, double, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = optcon::TermBase<STATE_DIM, CONTROL_DIM, double, double>;

    using RBDState = ct::rbd::RBDState<KINEMATICS::NJOINTS>;

    using state_matrix_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using control_matrix_t = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>;
    using control_state_matrix_t = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;


    //! default constructor
    TermTaskspaceGeometricJacobian() = default;

    //! constructor using a quaternion for orientation
    TermTaskspaceGeometricJacobian(size_t eeInd,
        const Eigen::Matrix3d& Qpos,
        const Eigen::Matrix3d& Qrot,
        const core::StateVector<3>& w_pos_des,
        const Eigen::Quaterniond& w_q_des,
        const std::string& name = "TermTaskSpace")
        : BASE(name), eeInd_(eeInd), Q_pos_(Qpos), Q_rot_(Qrot)
    {
        setReferencePosition(w_pos_des);
        setReferenceOrientation(w_q_des);
    }

    //! constructor taking a full RigidBodyPose
    TermTaskspaceGeometricJacobian(size_t eeInd,
        const Eigen::Matrix3d& Qpos,
        const Eigen::Matrix3d& Qrot,
        const ct::rbd::RigidBodyPose& rbdPose,
        const std::string& name = "TermTaskSpace")
        // delegate constructor
        : TermTaskspaceGeometricJacobian(eeInd,
              Qpos,
              Qrot,
              rbdPose.position().toImplementation(),
              rbdPose.getRotationQuaternion().toImplementation(),
              name)
    {
    }

    //! constructor using Euler angles for orientation
    TermTaskspaceGeometricJacobian(size_t eeInd,
        const Eigen::Matrix3d& Qpos,
        const Eigen::Matrix3d& Qrot,
        const core::StateVector<3>& w_pos_des,
        const Eigen::Matrix3d& eulerXyz,
        const std::string& name = "TermTaskSpace")
        // delegate constructor
        : TermTaskspaceGeometricJacobian(eeInd,
              Qpos,
              Qrot,
              w_pos_des,
              Eigen::Quaterniond(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ())),
              name)
    {
    }

    //! constructor which sets the target pose to dummy values
    TermTaskspaceGeometricJacobian(size_t eeInd,
        const Eigen::Matrix3d& Qpos,
        const Eigen::Matrix3d& Qrot,
        const std::string& name = "TermTaskSpace")
        : BASE(name), eeInd_(eeInd), Q_pos_(Qpos), Q_rot_(Qrot)
    {
        // arbitrary dummy values
        Eigen::Quaterniond w_q_des(0.0, 0.0, 0.0, 1.0);
        setReferenceOrientation(w_q_des);
        setReferencePosition(core::StateVector<3>::Zero());
    }

    //! construct this term with info loaded from a configuration file
    TermTaskspaceGeometricJacobian(const std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    TermTaskspaceGeometricJacobian(const TermTaskspaceGeometricJacobian& arg)
        : BASE(arg),
          eeInd_(arg.eeInd_),
          kinematics_(KINEMATICS()),
          Q_pos_(arg.Q_pos_),
          Q_rot_(arg.Q_rot_),
          x_ref_(arg.x_ref_),
          R_ref_(arg.R_ref_)
    {
    }

    //! destructor
    virtual ~TermTaskspaceGeometricJacobian() = default;

    //! deep cloning
    virtual TermTaskspaceGeometricJacobian<KINEMATICS, STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new TermTaskspaceGeometricJacobian(*this);
    }

    //! evaluate
    virtual double evaluate(const Eigen::Matrix<double, STATE_DIM, 1>& x,
        const Eigen::Matrix<double, CONTROL_DIM, 1>& u,
        const double& t) override
    {
        // transform the robot state vector into a CT RBDState
        RBDState rbdState;
        rbdState.jointPositions() = x.template head<KINEMATICS::NJOINTS>();

        // position difference in world frame
        Eigen::Matrix<double, 3, 1> xCurr = kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions()).toImplementation();
        Eigen::Matrix<double, 3, 1> xDiff = xCurr - x_ref_;

        // compute the cost from the position error
        double pos_cost = 0.5 * (xDiff.transpose() * Q_pos_ * xDiff)(0, 0);

        // get current end-effector rotation in world frame
        Eigen::Matrix<double, 3, 3> ee_rot = kinematics_.getEERotInBase(eeInd_, rbdState.jointPositions());
        Eigen::Quaterniond orientation(ee_rot);
        Eigen::Quaterniond orientation_d_(R_ref_);
        Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
        // convert to axis angle
        Eigen::AngleAxis<double> error_quaternion_angle_axis(error_quaternion);
        // compute "orientation position_error"
        Eigen::Matrix<double, 3, 1> qDiff = error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

        double rot_cost = 0.5 * (qDiff.transpose() * Q_rot_ * qDiff)(0, 0);

        return pos_cost + rot_cost;
    }


    //! compute derivative of this cost term w.r.t. the state
    virtual core::StateVector<STATE_DIM> stateDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        core::StateVector<STATE_DIM> grad;
        grad.setZero();

        RBDState rbdState;
        rbdState.jointPositions() = x.template head<KINEMATICS::NJOINTS>();

        Eigen::Matrix<double, 6, KINEMATICS::NJOINTS> J = kinematics_.getJacobianBaseEEbyId(eeInd_, rbdState);
        Eigen::Matrix<double, 3, KINEMATICS::NJOINTS> J_pos = J.template bottomRows<3>();
        Eigen::Matrix<double, 3, KINEMATICS::NJOINTS> J_q = J.template topRows<3>();

        // position difference in world frame
        Eigen::Matrix<double, 3, 1> xCurr =
            kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions()).toImplementation();
        Eigen::Matrix<double, 3, 1> xDiff = xCurr - x_ref_;

        grad.template head<KINEMATICS::NJOINTS>() += J_pos.transpose() * Q_pos_ * xDiff;


        // get current end-effector rotation in world frame
        Eigen::Matrix<double, 3, 3> ee_rot = kinematics_.getEERotInBase(eeInd_, rbdState.jointPositions());
        Eigen::Quaterniond orientation(ee_rot);
        Eigen::Quaterniond orientation_d_(R_ref_);
        Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
        // convert to axis angle
        Eigen::AngleAxis<double> error_quaternion_angle_axis(error_quaternion);
        // compute "orientation position_error"
        Eigen::Matrix<double, 3, 1> qDiff = error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

        grad.template head<KINEMATICS::NJOINTS>() += J_q.transpose() * Q_rot_ * qDiff;

        return grad;
    }

    //! compute derivative of this cost term w.r.t. the control input
    virtual core::ControlVector<CONTROL_DIM> controlDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        return core::ControlVector<CONTROL_DIM>::Zero();
    }

    //! compute second order derivative of this cost term w.r.t. the state
    virtual state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        RBDState rbdState;
        rbdState.jointPositions() = x.template head<KINEMATICS::NJOINTS>();

        Eigen::Matrix<double, 6, KINEMATICS::NJOINTS> J = kinematics_.getJacobianBaseEEbyId(eeInd_, rbdState);
        Eigen::Matrix<double, 3, KINEMATICS::NJOINTS> J_pos = J.template bottomRows<3>();
        Eigen::Matrix<double, 3, KINEMATICS::NJOINTS> J_q = J.template topRows<3>();

        state_matrix_t Q = state_matrix_t::Zero();
        Q.template topLeftCorner<KINEMATICS::NJOINTS, KINEMATICS::NJOINTS>() += J_pos.transpose() * Q_pos_ * J_pos;
        Q.template topLeftCorner<KINEMATICS::NJOINTS, KINEMATICS::NJOINTS>() += J_q.transpose() * Q_rot_ * J_q;

        return Q;
    }

    //! compute second order derivative of this cost term w.r.t. the control input
    virtual control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        return control_matrix_t::Zero();
    }

    //! compute the cross-term derivative (state-control) of this cost function term
    virtual control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        return control_state_matrix_t::Zero();
    }


    //! load term information from configuration file (stores data in member variables)
    void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        if (verbose)
            std::cout << "Loading TermTaskspaceGeometricJacobian from file " << filename << std::endl;

        ct::optcon::loadScalarCF(filename, "eeId", eeInd_, termName);
        ct::optcon::loadMatrixCF(filename, "Q_rot", Q_rot_, termName);
        ct::optcon::loadMatrixCF(filename, "Q_pos", Q_pos_, termName);

        Eigen::Matrix<double, 3, 1> w_p_ref;
        ct::optcon::loadMatrixCF(filename, "x_des", w_p_ref, termName);
        setReferencePosition(w_p_ref);


        // try loading euler angles
        if (verbose)
            std::cout << "trying to load Euler angles" << std::endl;
        try
        {
            Eigen::Vector3d eulerXyz;
            ct::optcon::loadMatrixCF(filename, "eulerXyz_des", eulerXyz, termName);
            Eigen::Quaternion<double> quat_des(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                                               Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ()));
            setReferenceOrientation(quat_des);
            if (verbose)
                std::cout << "Read desired Euler Angles Xyz as  = \n" << eulerXyz.transpose() << std::endl;
        } catch (const std::exception& e)
        {
            throw std::runtime_error(
                "Failed to load TermTaskspaceGeometricJacobian, could not find a desired end effector orientation in "
                "file.");
        }

        if (verbose)
        {
            std::cout << "Read eeId as eeId = \n" << eeInd_ << std::endl;
            std::cout << "Read Q_pos as Q_pos = \n" << Q_pos_ << std::endl;
            std::cout << "Read Q_rot as Q_rot = \n" << Q_rot_ << std::endl;
            std::cout << "Read x_des as x_des = \n" << getReferencePosition().transpose() << std::endl;
        }
    }


    //! retrieve reference position in world frame
    const Eigen::Matrix<double, 3, 1> getReferencePosition() const { return x_ref_; }
    //! set the end-effector reference position (in base coordinates)
    void setReferencePosition(Eigen::Matrix<double, 3, 1> p_ref) { x_ref_ = p_ref; }
    //! retrieve reference ee orientation in world frame
    const Eigen::Quaterniond getReferenceOrientation() const { return Eigen::Quaterniond(R_ref_); }
    //! set desired end-effector orientation in world frame
    void setReferenceOrientation(const Eigen::Matrix<double, 3, 3>& w_R_ref) { R_ref_ = w_R_ref; }

    //! set desired end-effector orientation in world frame
    void setReferenceOrientation(const Eigen::Quaterniond& w_q_des)
    {
        setReferenceOrientation(w_q_des.normalized().toRotationMatrix());
    }

    //! return the reference pose as RigidBodyPose
    const ct::rbd::RigidBodyPose getReferencePose() const
    {
        return ct::rbd::RigidBodyPose(getReferenceOrientation(), getReferencePosition());
    }


private:
    //! index of the end-effector
    size_t eeInd_;

    //! the robot kinematics
    KINEMATICS kinematics_;

    //! weighting matrix for the task-space position
    Eigen::Matrix3d Q_pos_;

    //! weighting matrix for orientation error
    Eigen::Matrix3d Q_rot_;

    Eigen::Matrix<double, 3, 1> x_ref_;  // ref position
    Eigen::Matrix<double, 3, 3> R_ref_;  // ref rotation
};


}  // namespace rbd
}  // namespace ct
