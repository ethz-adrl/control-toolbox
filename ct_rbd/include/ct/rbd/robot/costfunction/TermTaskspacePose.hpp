/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/utility/utilities.hpp>

#include <iit/rbd/traits/TraitSelector.h>
#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>


namespace ct {
namespace rbd {


/*!
 * \brief A costfunction term that defines a cost on a task-space pose
 *
 *  @todo add velocity to term
 *
 * \tparam KINEMATICS kinematics of the system
 * \tparam FB true if system is a floating base robot
 * \tparam STATE_DIM state dimensionality of the system
 * \tparam CONTROL_DIM control dimensionality of the system
 *
 *
 * todo:
 * - we can specify poses either in terms of euler angles or in terms of quaternions
 * - we should do all he math in quaternions
 * - a possible cost: the norm of the difference quaternion  |q_diff| ? the weighted norm q_diff' Q_quat q_diff ?
 *
 */
template <class KINEMATICS, bool FB, size_t STATE_DIM, size_t CONTROL_DIM>
class TermTaskspacePose : public optcon::TermBase<STATE_DIM, CONTROL_DIM, double, ct::core::ADCGScalar>
{
public:
    using SCALAR = ct::core::ADCGScalar;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! the trivial constructor is explicitly forbidden
    TermTaskspacePose() = delete;

    //! constructor
    TermTaskspacePose(size_t eeInd,
        const Eigen::Matrix<double, 3, 3>& Qpos,
        const core::StateVector<3, double>& pos_des = core::StateVector<3, double>::Zero(),
        const std::string& name = "TermTaskSpace")
        : optcon::TermBase<STATE_DIM, CONTROL_DIM, double, ct::core::ADCGScalar>(name),
          eeInd_(eeInd),
          QTaskSpace_(Qpos),
          pos_ref_(pos_des)
    {
        // Checks whether STATE_DIM has the appropriate size.
        //  2 * (FB * 6 + KINEMATICS::NJOINTS)) represents a floating base system with euler angles
        //  2 * (FB * 6 + KINEMATICS::NJOINTS) + 1 representing a floating base system with quaternion angles
        static_assert(
            (STATE_DIM == 2 * (FB * 6 + KINEMATICS::NJOINTS)) || (STATE_DIM == 2 * (FB * 6 + KINEMATICS::NJOINTS) + 1),
            "STATE_DIM does not have appropriate size.");
    }

    //! construct this term with info loaded from a configuration file
    TermTaskspacePose(std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    TermTaskspacePose(const TermTaskspacePose& arg)
        : eeInd_(arg.eeInd_), kinematics_(KINEMATICS()), QTaskSpace_(arg.QTaskSpace_), pos_ref_(arg.pos_ref_)
    {
    }

    //! destructor
    virtual ~TermTaskspacePose() {}
    //! deep cloning
    TermTaskspacePose<KINEMATICS, FB, STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new TermTaskspacePose(*this);
    }
    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override
    {
        return evalLocal<SCALAR>(x, u, t);
    }

    //! we overload the evaluateCppadCg method
    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        return evalLocal<ct::core::ADCGScalar>(x, u, t);
    }

    //! load term information from configuration file (stores data in member variables)
    void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        ct::optcon::loadScalarCF(filename, "eeId", eeInd_, termName);
        ct::optcon::loadMatrixCF(filename, "Q", QTaskSpace_, termName);
        ct::optcon::loadMatrixCF(filename, "x_des", pos_ref_, termName);

        if (verbose)
        {
            std::cout << "Read eeId as eeId = \n" << eeInd_ << std::endl;
            std::cout << "Read Q as Q = \n" << QTaskSpace_ << std::endl;
            std::cout << "Read x_des as x_des = \n" << pos_ref_.transpose() << std::endl;
        }
    }


private:
    //! a templated evaluate() method
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x, const Eigen::Matrix<SC, CONTROL_DIM, 1>& u, const SC& t)
    {
        // transform the robot state vector into a CT RBDState
        tpl::RBDState<KINEMATICS::NJOINTS, SC> rbdState = setStateFromVector<FB>(x);

        // compute the current end-effector position
        tpl::RigidBodyPose<SC> ee_pose =
            kinematics_.getEEPoseInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions());

        // todo do we have a method getEEPoseInWorld? and if yes, can it return both quaternions and euler angles?
        // todo does the rigid body pose naturally implement a difference measure? -> might want to employ that

        // compute the position error
//        Eigen::Matrix<SC, 3, 1> xDiff = ee_pos - pos_ref_.template cast<SC>();
//
//        // compute the cost based on the position error
//        SC cost = (xDiff.transpose() * QTaskSpace_.template cast<SC>() * xDiff)(0, 0);

        SC cost;

        return cost;
    }


    //! computes RBDState in case the user supplied a floating-base robot
    template <bool T>
    tpl::RBDState<KINEMATICS::NJOINTS, SCALAR> setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        typename std::enable_if<T, bool>::type = true)
    {
        tpl::RBDState<KINEMATICS::NJOINTS, SCALAR> rbdState;
        rbdState.fromStateVectorEulerXyz(x);

        return rbdState;
    }

    //! computes RBDState in case the user supplied a fixed-base robot
    template <bool T>
    tpl::RBDState<KINEMATICS::NJOINTS, SCALAR> setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        typename std::enable_if<!T, bool>::type = true)
    {
        tpl::RBDState<KINEMATICS::NJOINTS, SCALAR> rbdState;
        rbdState.joints() = x;
        return rbdState;
    }

    //! index of the end-effector in question
    size_t eeInd_;

    //! the robot kinematics
    KINEMATICS kinematics_;

    //! weighting matrix for the task-space position
    Eigen::Matrix<double, 3, 3> QTaskSpace_;

    //! reference position in task-space
    Eigen::Matrix<double, 3, 1> pos_ref_;
};


}  // namespace rbd
}  // namespace ct
