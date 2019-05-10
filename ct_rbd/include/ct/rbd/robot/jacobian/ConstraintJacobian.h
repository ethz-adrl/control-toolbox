/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
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

    typedef RBDState<NJOINTS, SCALAR> RBDState_t;
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


    void getJacobianOriginDerivativeNumdiff(const RBDState_t& state, jacobian_t& dJdt)
    {
        /*
		 * Takes the numdiff (single sided) of getContactJacobian.  derivative is dJ/dt = dJ/dq * dq/dt
		 * Because the contactJacobian (defined in the base frame) is independent of floating base coordinates
     * -> only numdiff against joints
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

    virtual void getJacobianOriginDerivative(const RBDState_t& state, jacobian_t& dJdt) override
    {
        /*
         *  Analytic approach for Jacobian derivative
         *  Derived from https://doi.org/10.1007/s11044-012-9334-7
         */
        // Temp objects
        Eigen::Matrix<SCALAR, 3, NJOINTS + 6> dJdt_eeId;
        Eigen::Matrix<SCALAR, 6, NJOINTS> Jc_geometric;
        Eigen::Matrix<SCALAR, 3, NJOINTS> Jc_Rotational, Jc_Translational, dJdt_joints;
        Eigen::Matrix<SCALAR, 3, 1> dJidqj;
        Eigen::Matrix<SCALAR, 3, 6> dJdt_base;
        Eigen::Matrix<SCALAR, 3, 1> eeVelocityWrtBase;  // Time derivative of the postion vector from base to EE

        dJdt_base.setZero();
        dJdt.setZero();
        for (size_t ee = 0; ee < ee_indices_.size(); ee++)
        {
            if (eeInContact_[ee_indices_[ee]])
            {
                // Collect current contact Jacobians
                Jc_geometric =
                    kinematics_.robcogen().getJacobianBaseEEbyId(ee_indices_[ee], state.joints().getPositions());
                Jc_Rotational = Jc_geometric.template topRows<3>();
                Jc_Translational = Jc_geometric.template bottomRows<3>();

                // Compute dJdt for the joint columns
                dJdt_joints.setZero();
                for (size_t i = 0; i < NJOINTS; i++)
                {  // Loop over columns i of dJdt_joints
                    for (size_t j = 0; j < NJOINTS; j++)
                    {  // Loop over derivative w.r.t each joint j and sum
                        if (i >= j)
                        {
                            dJidqj = (Jc_Rotational.template block<3, 1>(0, j))
                                         .template cross(Jc_Translational.template block<3, 1>(0, i));
                        }
                        else
                        {  // i < j
                            dJidqj = (Jc_Rotational.template block<3, 1>(0, i))
                                         .template cross(Jc_Translational.template block<3, 1>(0, j));
                        }
                        dJdt_joints.template block<3, 1>(0, i) += dJidqj * state.joints().getVelocities()(j);
                    }
                }

                // Add the base columns: [-skew(v), 0_(3x3)]
                eeVelocityWrtBase = Jc_Translational * state.joints().getVelocities();
                dJdt_base(1, 2) = eeVelocityWrtBase(0);  // x
                dJdt_base(2, 1) = -eeVelocityWrtBase(0);
                dJdt_base(0, 2) = -eeVelocityWrtBase(1);  // y
                dJdt_base(2, 0) = eeVelocityWrtBase(1);
                dJdt_base(0, 1) = eeVelocityWrtBase(2);  // z
                dJdt_base(1, 0) = -eeVelocityWrtBase(2);

                // Fill in the rows of the full dJdt
                dJdt.template block<3, 6>(ee_indices_[ee] * 3, 0) = dJdt_base;
                dJdt.template block<3, NJOINTS>(ee_indices_[ee] * 3, 6) = dJdt_joints;
            }
        }
    }

    virtual void getJacobianOrigin(const RBDState_t& state, jacobian_t& Jc) override
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
