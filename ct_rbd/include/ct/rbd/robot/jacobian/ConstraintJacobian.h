/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/robot/jacobian/OperationalJacobianBase.h>
#include <ct/rbd/state/RBDState.h>

#include "FrameJacobian.h"

/**
 * \ingroup OS
 */
namespace ct {
namespace rbd {
namespace tpl {

template <typename Kinematics, size_t OUTPUTS, size_t NJOINTS, typename SCALAR>
class ConstraintJacobian : public OperationalJacobianBase<OUTPUTS, NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef tpl::RBDState<NJOINTS, SCALAR> RBDState_t;
    typedef typename OperationalJacobianBase<OUTPUTS, NJOINTS, SCALAR>::jacobian_t jacobian_t;
    typedef Eigen::Matrix<SCALAR, 3, 3> Matrix3s;

    ConstraintJacobian(){};

    virtual ~ConstraintJacobian(){};

    void SetBlockZero(const int& row)
    {
        // todo: check if needed
        this->J();
        this->J_.template block<3, NJOINTS + 6>(row, 0).setZero();
    }

    static const size_t BASE_DOF = 6;

    size_t c_size_ = 0;
    std::vector<int> ee_indices_;
    std::array<bool, Kinematics::NUM_EE> eeInContact_;


    virtual void getJacobianOriginDerivative(const RBDState_t& state, jacobian_t& dJdt)
    {
        /*
		 * Takes the numdiff (single sided) of getContactJacobian.  derivative is dJ/dt = dJ/dq * dq/dt
		 * Because the contactJacobian (defined in the base frame) is independent of floating base coordinates -> only numdiff against joints
		 */
        jacobian_t Jc0, Jc1;
        RBDState_t state1 = state;
        getJacobianOrigin(state, Jc0);

        SCALAR eps_ = sqrt(Eigen::NumTraits<SCALAR>::epsilon());

        dJdt.setZero();
        for (size_t i = 0; i < NJOINTS; i++)  //
        {
            SCALAR h = eps_ * std::max(core::tpl::TraitSelector<SCALAR>::Trait::fabs(state.joints().getPositions()(i)),
                                  SCALAR(1.0));  // h = eps_ * max(abs(qj(i)), 1.0)
            state1.joints().getPositions()(i) += h;
            getJacobianOrigin(state1, Jc1);
            dJdt += (Jc1 - Jc0) / h * state.joints().getVelocities()(i);
            state1.joints().getPositions()(i) = state.joints().getPositions()(i);
        }
    }

    virtual void getJacobianOrigin(const RBDState_t& state, jacobian_t& Jc)
    {
        Jc.setZero();
        for (size_t ee = 0; ee < ee_indices_.size(); ee++)
        {
            if (eeInContact_[ee_indices_[ee]])
            {
                Eigen::Matrix<SCALAR, 3, NJOINTS + 6> J_eeId;
                kindr::Position<SCALAR, 3> eePosition =
                    kinematics_.getEEPositionInBase(ee_indices_[ee], state.joints().getPositions());
                Eigen::Matrix<SCALAR, 3, NJOINTS> J_single =
                    kinematics_.robcogen()
                        .getJacobianBaseEEbyId(ee_indices_[ee], state.joints().getPositions())
                        .template bottomRows<3>();
                FrameJacobian<NJOINTS, SCALAR>::FromBaseJacToInertiaJacTranslation(
                    Matrix3s::Identity(), eePosition.toImplementation(), J_single, J_eeId);
                Jc.template block<3, NJOINTS + 6>(ee_indices_[ee] * 3, 0) = J_eeId;
            }
        }
    }


private:
    Kinematics kinematics_;
};

}  // namespace tpl

template <typename Kinematics, size_t OUTPUTS, size_t NJOINTS>
using ConstraintJacobian = tpl::ConstraintJacobian<Kinematics, OUTPUTS, NJOINTS, double>;


} /* namespace rbd*/
} /* namespace ct*/
