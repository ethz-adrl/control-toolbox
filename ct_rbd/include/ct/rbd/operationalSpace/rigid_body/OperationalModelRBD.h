/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/robot/jacobian/FrameJacobian.h>
#include <ct/rbd/robot/kinematics/EndEffector.h>
#include "OperationalModelBase.h"

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \ingroup OS
 * \brief This is a class for expressing the RBD equations of an articulated robot as an operational model.
 * It uses the RBDContainer class as an interface to access the generated code for an articulated robot.
 */
template <class RBDContainer, size_t NUM_CONTACTPOINTS, typename SCALAR = double>
class OperationalModelRBD
    : public OperationalModelBase<RBDContainer::NJOINTS + 6, RBDContainer::NJOINTS, NUM_CONTACTPOINTS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR, 3, 3> Matrix3s;
    typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;

    enum
    {
        NUM_OUTPUTS = RBDContainer::NJOINTS + 6,
        NUM_JOINTS = RBDContainer::NJOINTS,
    };

    typedef std::shared_ptr<OperationalModelRBD<RBDContainer, NUM_CONTACTPOINTS>> ptr;
    typedef OperationalModelBase<NUM_OUTPUTS, NUM_JOINTS, NUM_CONTACTPOINTS> Base;
    typedef typename Base::state_t state_t;

    OperationalModelRBD(const typename RBDContainer::Ptr_t& rbdContainerPtr,
        const std::array<EndEffector<NUM_JOINTS>, NUM_CONTACTPOINTS>& endEffectorArray)
        : rbdContainerPtr_(rbdContainerPtr), endEffectorArray_(endEffectorArray)
    {
    }

    ~OperationalModelRBD() {}
    /*!
	 * This method updates the class member variables using the current state.
	 * @param state The current state
	 */
    void update(const state_t& state) override
    {
        Base::state_ = state;

        // rotation matrix
        Matrix3s o_R_b = state.basePose().getRotationMatrix().toImplementation();

        // Mass matrix
        Base::M_ = rbdContainerPtr_->jSim().update(state.jointPositions());
        Base::MInverse_ = Base::M_.ldlt().solve(Eigen::MatrixXd::Identity(18, 18));

        rbdContainerPtr_->inverseDynamics().setJointStatus(state.jointPositions());
        Eigen::Matrix<SCALAR, 6, 1> baseWrench;
        Eigen::Matrix<SCALAR, 12, 1> jointForces;

        // centrifugal vector
        Eigen::Matrix<SCALAR, 6, 1> trunkVel;
        trunkVel << state.baseLocalAngularVelocity().toImplementation(), state.baseLinearVelocity().toImplementation();
        rbdContainerPtr_->inverseDynamics().C_terms_fully_actuated(
            baseWrench, jointForces, trunkVel, state.jointVelocities());
        Base::C_ << baseWrench, jointForces;

        // gravity vector
        rbdContainerPtr_->inverseDynamics().G_terms_fully_actuated(
            baseWrench, jointForces, state.basePose().computeGravityB6D());
        Base::G_ << baseWrench, jointForces;

        // Selection matrix
        Base::S_ << Eigen::MatrixXd::Zero(12, 6), Eigen::MatrixXd::Identity(12, 12);

        // contact jacobians
        for (size_t j = 0; j < NUM_CONTACTPOINTS; j++)
        {
            Vector3s b_r_f = rbdContainerPtr_->getEEPositionInBase(j, state.jointPositions()).toImplementation();
            Eigen::Matrix<SCALAR, 3, NUM_JOINTS> b_J_f =
                rbdContainerPtr_->getJacobianBaseEEbyId(j, state.jointPositions()).template bottomRows<3>();

            FrameJacobian<NUM_JOINTS, SCALAR>::FromBaseJacToInertiaJacTranslation(o_R_b, b_r_f, b_J_f, Base::AllJc_[j]);
        }  // end of j loop
    }

private:
    typename RBDContainer::Ptr_t rbdContainerPtr_;
    std::array<EndEffector<NUM_JOINTS>, NUM_CONTACTPOINTS> endEffectorArray_;
};

}  // namespace tpl

template <class RBDContainer, size_t NUM_CONTACTPOINTS>
using OperationalModelRBD = tpl::OperationalModelRBD<RBDContainer, NUM_CONTACTPOINTS, double>;

}  // end of rbd namespace
}  // end of os namespace
