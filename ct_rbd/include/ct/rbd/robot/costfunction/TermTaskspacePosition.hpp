/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/utility/utilities.hpp>

#include <iit/rbd/traits/TraitSelector.h>
#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>


namespace ct {
namespace rbd {


/*!
 * \brief A costfunction term that defines a cost on a taskspace position.
 * \warning This term does not consider orientations. For a full pose cost function, see TermTaskspacePose.h
 *
 * This cost function adds a quadratic penalty on the position offset of an endeffector to a desired position
 *  @todo add velocity to term
 *
 * \tparam KINEMATICS kinematics of the system
 * \tparam FB true if system is a floating base robot
 * \tparam STATE_DIM state dimensionality of the system
 * \tparam CONTROL_DIM control dimensionality of the system
 */
template <class KINEMATICS, bool FB, size_t STATE_DIM, size_t CONTROL_DIM>
class TermTaskspacePosition : public optcon::TermBase<STATE_DIM, CONTROL_DIM, double, ct::core::ADCGScalar>
{
public:
    using SCALAR = ct::core::ADCGScalar;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! the trivial constructor is explicitly forbidden
    TermTaskspacePosition() = delete;

    //! constructor
    TermTaskspacePosition(size_t eeInd,
        const Eigen::Matrix<double, 3, 3>& Q,
        const core::StateVector<3, double>& pos_des,
        const std::string& name = "TermTaskSpace")
        : optcon::TermBase<STATE_DIM, CONTROL_DIM, double, ct::core::ADCGScalar>(name),
          eeInd_(eeInd),
          QTaskSpace_(Q),
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
    TermTaskspacePosition(std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    TermTaskspacePosition(const TermTaskspacePosition& arg)
        : eeInd_(arg.eeInd_), kinematics_(KINEMATICS()), QTaskSpace_(arg.QTaskSpace_), pos_ref_(arg.pos_ref_)
    {
    }

    //! destructor
    virtual ~TermTaskspacePosition() {}
    //! deep cloning
    TermTaskspacePosition<KINEMATICS, FB, STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new TermTaskspacePosition(*this);
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
        RBDState<KINEMATICS::NJOINTS, SC> rbdState = setStateFromVector<FB>(x);

        Eigen::Matrix<SC, 3, 1> xDiff =
            kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions())
                .toImplementation() -
            pos_ref_.template cast<SC>();

        SC cost = (xDiff.transpose() * QTaskSpace_.template cast<SC>() * xDiff)(0, 0);
        return cost;
    }


    //! computes RBDState in case the user supplied a floating-base robot
    template <bool T>
    RBDState<KINEMATICS::NJOINTS, SCALAR> setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        typename std::enable_if<T, bool>::type = true)
    {
        RBDState<KINEMATICS::NJOINTS, SCALAR> rbdState;
        rbdState.fromStateVectorEulerXyz(x);

        return rbdState;
    }

    //! computes RBDState in case the user supplied a fixed-base robot
    template <bool T>
    RBDState<KINEMATICS::NJOINTS, SCALAR> setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        typename std::enable_if<!T, bool>::type = true)
    {
        RBDState<KINEMATICS::NJOINTS, SCALAR> rbdState;
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

#endif
