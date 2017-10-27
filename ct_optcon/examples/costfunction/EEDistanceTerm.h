/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef EE_DISTANCE_TERM_EXAMPLE_H_
#define EE_DISTANCE_TERM_EXAMPLE_H_

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

#include <ct/optcon/costfunction/term/TermBase.hpp>

#include "Kinematics.h"


namespace ct {
namespace optcon {
namespace example {

/*the system consists of a little base, movable in x and y direction, and a 6-dof arm attached to it*/
const size_t stateDim_planar = 2;    // state		robot base x,y
const size_t controlDim_planar = 8;  // control 		robot base input velocities, robot joint input positions


typedef CppAD::AD<double> AD_scalar_t;
typedef Eigen::Matrix<AD_scalar_t, 3, 1> AD_position_t;
typedef Eigen::Matrix<AD_scalar_t, 6, 1> AD_joint_state_t;
typedef Eigen::Matrix<AD_scalar_t, 3, 3> AD_rotation_matrix_t;
typedef Eigen::Matrix<AD_scalar_t, stateDim_planar, 1> AD_state_vector_t;
typedef Eigen::Matrix<AD_scalar_t, controlDim_planar, 1> AD_control_vector_t;
typedef Eigen::Matrix<AD_scalar_t, stateDim_planar, stateDim_planar> AD_state_matrix_t;


/*!
 * This costfunction term penalizes the distance between the end-effector defineded through "Kinematics.h" and a desired goal position
 * \example EEDistanceTerm.h
 */
class EEDistanceTerm : public ct::TermBase<stateDim_planar, controlDim_planar, CppAD::AD<double>>
{
public:
    EEDistanceTerm(const Eigen::Vector3d& desEEpos, const Eigen::Matrix<double, 3, 3>& Q)
    {
        for (size_t i = 0; i < 3; i++)
        {
            EE_desired_(i) = desEEpos(i);
        }

        for (size_t j = 0; j < 3; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                Q_(j, i) = Q(j, i);
            }
        }

        std::cout << "Q for ee term" << std::endl << Q_ << std::endl;
        std::cout << "ee_desired in ee term" << std::endl << EE_desired_ << std::endl;
    }

    EEDistanceTerm(const EEDistanceTerm& arg)
        : ct::TermBase<stateDim_planar, controlDim_planar, CppAD::AD<double>>(arg),
          Q_(arg.Q_),
          EE_desired_(arg.EE_desired_),
          type_fr_link0_X_ee_(arg.type_fr_link0_X_ee_)
    {
    }

    std::shared_ptr<ct::TermBase<stateDim_planar, controlDim_planar, AD_scalar_t>> clone() const override
    {
        return std::shared_ptr<ct::TermBase<stateDim_planar, controlDim_planar, AD_scalar_t>>(
            new EEDistanceTerm(*this));
    }

    ~EEDistanceTerm() {}
    AD_scalar_t evaluateIntermediate(const AD_state_vector_t& x, const AD_control_vector_t& u, double t) override
    {
        AD_joint_state_t jointState = u.segment(2, 6);

        AD_position_t EE_pos_diff = EE_desired_ - getEEPositionsWorld(x, jointState);

        AD_scalar_t cost_EE_pos = (EE_pos_diff.transpose() * Q_ * EE_pos_diff);

        return cost_EE_pos;
    }


    AD_position_t getEEPositionsWorld(const AD_state_vector_t& basePose, const AD_joint_state_t& q)
    {
        AD_position_t B_rEE = type_fr_link0_X_ee_.get_position(q);

        // transform from base to world

        AD_position_t W_r_W0_B0;
        W_r_W0_B0(0) = basePose(0);  // x coordinate base
        W_r_W0_B0(1) = basePose(1);  // y coordinate base
        W_r_W0_B0(2) = 0;            // z coordinate base

        AD_position_t posWorld = W_r_W0_B0 + B_rEE;

        return posWorld;
    }


private:
    AD_rotation_matrix_t Q_;
    AD_position_t EE_desired_;

    AD_Type_fr_link0_X_ee<AD_scalar_t> type_fr_link0_X_ee_;
};

}  // namespace example
}  // namespace optcon
}  // namespace ct


#endif
