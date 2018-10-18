/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich
Licensed under Apache2 license (see LICENSE file in main directory)
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
        const double& Qrot,
        const core::StateVector<3>& w_pos_des,
        const Eigen::Quaterniond& w_q_des,
        const std::string& name = "TermTaskSpace")
        : BASE(name), eeInd_(eeInd), Q_pos_(Qpos), Q_rot_(Qrot)
    {
        setReferencePosition(w_pos_des);
        setReferenceOrientation(w_q_des);
        setup();
    }

    //! constructor taking a full RigidBodyPose
    TermTaskspaceGeometricJacobian(size_t eeInd,
        const Eigen::Matrix3d& Qpos,
        const double& Qrot,
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
        const double& Qrot,
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
        const double& Qrot,
        const std::string& name = "TermTaskSpace")
        : BASE(name), eeInd_(eeInd), Q_pos_(Qpos), Q_rot_(Qrot)
    {
        // arbitrary dummy values
        Eigen::Quaterniond w_q_des(0.0, 0.0, 0.0, 1.0);
        setReferenceOrientation(w_q_des);
        setReferencePosition(core::StateVector<3>::Zero());
        setup();
    }

    //! construct this term with info loaded from a configuration file
    TermTaskspaceGeometricJacobian(const std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    TermTaskspaceGeometricJacobian(const TermTaskspaceGeometricJacobian& arg)
        : BASE(arg), eeInd_(arg.eeInd_), kinematics_(KINEMATICS()), Q_pos_(arg.Q_pos_), Q_rot_(arg.Q_rot_)
    {
    }

    //! destructor
    virtual ~TermTaskspaceGeometricJacobian() {}
    //! deep cloning
    virtual TermTaskspaceGeometricJacobian<KINEMATICS, STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new TermTaskspaceGeometricJacobian(*this);
    }

    //! setup the AD Derivatives
    void setup() {}
    //! evaluate
    virtual double evaluate(const Eigen::Matrix<double, STATE_DIM, 1>& x,
        const Eigen::Matrix<double, CONTROL_DIM, 1>& u,
        const double& t) override
    {
        // transform the robot state vector into a CT RBDState
        RBDState rbdState;
        rbdState.jointPositions() = x.template head<KINEMATICS::NJOINTS>();

        // position difference in world frame
        Eigen::Matrix<double, 3, 1> xCurr =
            kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions()).toImplementation();
        Eigen::Matrix<double, 3, 1> xDiff = xCurr - x_ref_;

        // compute the cost from the position error
        double pos_cost = (xDiff.transpose() * Q_pos_ * xDiff)(0, 0);

        // get current end-effector rotation in world frame
        Eigen::Matrix<double, 3, 3> ee_rot = kinematics_.getEERotInBase(eeInd_, rbdState.jointPositions());

        // compute a measure for the difference between current rotation and desired rotation and compute cost based on the orientation error
        // for the intuition behind, consider the following posts:
        // https://math.stackexchange.com/a/87698
        // https://math.stackexchange.com/a/773635
//        Eigen::Matrix<double, 3, 3> ee_R_diff = w_R_ref.transpose() * ee_rot;

        // compute rotation penalty using the squared Frobenius norm of (R_diff-I)
//        double rot_cost = Q_rot_ * (ee_R_diff - Eigen::Matrix<double, 3, 3>::Identity()).squaredNorm();

        return pos_cost; // + rot_cost;
    }


    //! compute derivative of this cost term w.r.t. the state
    virtual core::StateVector<STATE_DIM> stateDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        core::StateVector<STATE_DIM> grad;
        grad.setZero();

        RBDState rbdState;
        rbdState.getJointPosition() = x.template head<KINEMATICS::NJOINTS>();
        Eigen::Matrix<double, 6, KINEMATICS::NJOINTS> J = kinematics_.getJacobianBaseEEbyId(eeInd_, rbdState);


        grad.template head<KINEMATICS::NJOINTS>() =
            2 * Q_pos_ *
            (kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions())
                    .toImplementation() -
                x_ref_)
                .transpose() *
            J;

        // todo implement !!

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
        throw std::runtime_error("stateSecondDerivative currently not defined for TermTaskspaceGeometricJacobian.");
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
        ct::optcon::loadScalarCF(filename, "Q_rot", Q_rot_, termName);
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

        // new initialization required
        setup();
    }


    //! retrieve reference position in world frame
    const Eigen::Matrix<double, 3, 1> getReferencePosition() const { return x_ref_; }
    //! set the end-effector reference position (in base coordinates)
    void setReferencePosition(Eigen::Matrix<double, 3, 1> p_ref) { x_ref_ = p_ref; }
    //! retrieve reference ee orientation in world frame
    const Eigen::Quaterniond getReferenceOrientation() const { return Eigen::Quaterniond(R_ref_); }
    //! set desired end-effector orientation in world frame
    void setReferenceOrientation(const Eigen::Matrix<double, 3, 3>& w_R_ref)
    {
        R_ref_ = w_R_ref;
        //        // transcribe the rotation matrix into the parameter vector
        //        const Eigen::Matrix<double, 9, 1> matVectorized(
        //            Eigen::Map<const Eigen::Matrix<double, 9, 1>>(w_R_ref.data(), 9));
        //        adParameterVector_.template segment<9>(STATE_DIM + CONTROL_DIM + 3) = matVectorized;
    }

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
    //    //! transcribe the 9x1 "rotation" segment from the AD parameter vector into a 3x3 matrix
    //    template <typename SC>
    //    const Eigen::Matrix<SC, 3, 3> extractReferenceRotationMatrix(
    //        const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& parameterVector) const
    //    {
    //        Eigen::Matrix<SC, 9, 1> matVectorized = parameterVector.template segment<9>(STATE_DIM + CONTROL_DIM + 3);
    //        Eigen::Matrix<SC, 3, 3> w_R_ee(Eigen::Map<Eigen::Matrix<SC, 3, 3>>(matVectorized.data(), 3, 3));
    //        return w_R_ee;
    //    }

    //    //! extract the state segment from the AD parameter vector
    //    template <typename SC>
    //    const Eigen::Matrix<SC, STATE_DIM, 1> extractStateVector(
    //        const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& parameterVector) const
    //    {
    //        return parameterVector.template segment<STATE_DIM>(0);
    //    }
    //
    //    //! extract the control segment from the AD parameter vector
    //    template <typename SC>
    //    const Eigen::Matrix<SC, 3, 1> extractReferencePosition(
    //        const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& parameterVector) const
    //    {
    //        return parameterVector.template segment<3>(STATE_DIM + CONTROL_DIM);
    //    }


    //! index of the end-effector
    size_t eeInd_;

    //! the robot kinematics
    KINEMATICS kinematics_;

    //! weighting matrix for the task-space position
    Eigen::Matrix3d Q_pos_;

    //! weighting factor for orientation error
    double Q_rot_;

    Eigen::Matrix<double, 3, 1> x_ref_;  // ref position
    Eigen::Matrix<double, 3, 3> R_ref_;   // ref rotation
};


}  // namespace rbd
}  // namespace ct
