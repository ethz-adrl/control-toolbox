/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/JointState.h>
#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class InverseKinematicsBase
{
public:
    virtual std::vector<typename JointState<NJOINTS, SCALAR>::Position> computeInverseKinematics(
        const tpl::RigidBodyPose<SCALAR> &eeBasePose,
        const std::vector<SCALAR> &freeJoints) const = 0;

    virtual std::vector<typename JointState<NJOINTS, SCALAR>::Position> computeInverseKinematics(
        const tpl::RigidBodyPose<SCALAR> &eeWorldPose,
        const tpl::RigidBodyPose<SCALAR> &baseWorldPose,
        const std::vector<SCALAR> &freeJoints) const = 0;

    virtual typename JointState<NJOINTS, SCALAR>::Position computeInverseKinematicsCloseTo(
        const tpl::RigidBodyPose<SCALAR> &eeWorldPose,
        const tpl::RigidBodyPose<SCALAR> &baseWorldPose,
        const std::vector<SCALAR> &freeJoints,
        const typename JointState<NJOINTS, SCALAR>::Position &queryJointPositions) const
    {
        auto solutions = computeInverseKinematics(eeWorldPose, baseWorldPose, freeJoints);
        if (!solutions.size()) throw std::runtime_error("No inverse kinematics solutions found!");

        typename JointState<NJOINTS, SCALAR>::Position closestPosition = solutions[0];
        double minNorm = (closestPosition - queryJointPositions).norm();

        for (int i = 1; i < solutions.size(); ++i)
        {
            if ((solutions[i] - queryJointPositions).norm() < minNorm)
            {
                minNorm = (solutions[i] - queryJointPositions).norm();
                closestPosition = solutions[i];
            }
        }
        return closestPosition;
    }
};

} /* namespace rbd */
} /* namespace ct */
