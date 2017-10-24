/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
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
 * \brief A costfunction term that defines a cost in task space
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
class TermTaskspace : public optcon::TermBase<STATE_DIM, CONTROL_DIM, double, ct::core::ADCGScalar>
{
public:
    typedef ct::core::ADCGScalar SCALAR;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! the trivial constructor is explicitly forbidden
    TermTaskspace() = delete;

    TermTaskspace(size_t eeInd,
        const Eigen::Matrix<double, 3, 3>& Q,
        const core::StateVector<3, double>& pos_des = core::StateVector<3, double>::Zero(),
        const std::string& name = "TermTaskSpace")
        : optcon::TermBase<STATE_DIM, CONTROL_DIM, double, ct::core::ADCGScalar>(name),
          eeInd_(eeInd),
          QTaskSpace_(Q),
          pos_ref_(pos_des)
    {
        currState_.setZero();
        // Checks whether STATE_DIM has the appropriate size.
        //  2 * (FB * 6 + KINEMATICS::NJOINTS)) represents a floating base system with euler angles
        //  2 * (FB * 6 + KINEMATICS::NJOINTS) + 1 representing a floating base system with quaternion angles

        static_assert(
            (STATE_DIM == 2 * (FB * 6 + KINEMATICS::NJOINTS)) || (STATE_DIM == 2 * (FB * 6 + KINEMATICS::NJOINTS) + 1),
            "STATE_DIM does not have appropriate size.");
    }

    //! load this term from a configuration file
    TermTaskspace(std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    TermTaskspace(const TermTaskspace& arg)
        : eeInd_(arg.eeInd_),
          currState_(arg.currState_),
          kinematics_(KINEMATICS()),
          QTaskSpace_(arg.QTaskSpace_),
          pos_ref_(arg.pos_ref_)
    {
    }

    virtual ~TermTaskspace() {}
    //! deep cloning
    TermTaskspace<KINEMATICS, FB, STATE_DIM, CONTROL_DIM>* clone() const override { return new TermTaskspace(*this); }
    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override
    {
        return evalLocal<SCALAR>(x, u, t);
    }

    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override
    {
        return evalLocal<ct::core::ADCGScalar>(x, u, t);
    }


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
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x, const Eigen::Matrix<SC, CONTROL_DIM, 1>& u, const SC& t)
    {
        setStateFromVector<FB>(x);

        Eigen::Matrix<SC, 3, 1> xDiff =
            kinematics_.getEEPositionInWorld(eeInd_, currState_.basePose(), currState_.jointPositions())
                .toImplementation() -
            pos_ref_.template cast<SC>();

        SCALAR cost = (xDiff.transpose() * QTaskSpace_.template cast<SC>() * xDiff)(0, 0);
        return cost;
    }


    template <bool T>
    void setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x, typename std::enable_if<T, bool>::type = true)
    {
        currState_.fromStateVectorRaw(x);
    }

    template <bool T>
    void setStateFromVector(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        typename std::enable_if<!T, bool>::type = true)
    {
        currState_.joints() = x;
    }

    size_t eeInd_;

    tpl::RBDState<KINEMATICS::NJOINTS, SCALAR> currState_;

    KINEMATICS kinematics_;

    Eigen::Matrix<double, 3, 3> QTaskSpace_;

    Eigen::Matrix<double, 3, 1> pos_ref_;
};


}  // namespace rbd
}  // namespace ct
