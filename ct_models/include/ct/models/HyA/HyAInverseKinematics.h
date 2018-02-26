/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE hya_ik
#include <ikfast.h>
#include <ct/core/core.h>
#include <ct/rbd/state/JointState.h>
#include <ct/models/HyA/HyAJointLimits.h>

#include <ct/rbd/robot/kinematics/InverseKinematicsBase.h>

using namespace ikfast;

namespace ct {
namespace rbd {

template <typename SCALAR = double>
class HyAInverseKinematics : InverseKinematicsBase<6, SCALAR>
{
public:
    virtual std::vector<typename JointState<6, SCALAR>::Position> computeInverseKinematics(
        const tpl::RigidBodyPose<SCALAR>& eeBasePose,
        const std::vector<SCALAR>& freeJoints = std::vector<SCALAR>()) const
    {
        IkSolutionList<double> solutions;

        if (size_t(hya_ik::GetNumFreeParameters()) != freeJoints.size())
            throw std::runtime_error("Error specifying free joints");

        // Data needs to be in row-major form.
        Eigen::Matrix<SCALAR, 3, 3, Eigen::RowMajor> eeBaseRotationRowMajor =
            eeBasePose.getRotationMatrix().toImplementation();
        hya_ik::ComputeIk(eeBasePose.position().toImplementation().data(), eeBaseRotationRowMajor.data(),
            freeJoints.size() > 0 ? freeJoints.data() : nullptr, solutions);

        size_t num_solutions = solutions.GetNumSolutions();
        std::vector<typename JointState<6, SCALAR>::Position> res;

        JointState<6, SCALAR> sol;
        for (size_t i = 0u; i < num_solutions; ++i)
        {
            const IkSolutionBase<double>& solution = solutions.GetSolution(i);
            solution.GetSolution(sol.getPositions().data(), freeJoints.size() > 0 ? freeJoints.data() : nullptr);
            sol.toUniquePosition(ct::models::HyA::jointLowerLimit());
            if (sol.checkPositionLimits(ct::models::HyA::jointLowerLimit(), ct::models::HyA::jointUpperLimit()))
                res.push_back(sol.getPositions());
        }

        return res;
    }

    virtual std::vector<typename JointState<6, SCALAR>::Position> computeInverseKinematics(
        const tpl::RigidBodyPose<SCALAR>& eeWorldPose,
        const tpl::RigidBodyPose<SCALAR>& baseWorldPose,
        const std::vector<SCALAR>& freeJoints = std::vector<SCALAR>()) const
    {
        return computeInverseKinematics(eeWorldPose.inReferenceFrame(baseWorldPose), freeJoints);
    }
};
} /* namespace rbd */
} /* namespace ct */
