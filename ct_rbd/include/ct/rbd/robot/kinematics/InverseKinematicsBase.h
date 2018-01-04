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
    virtual std::vector<typename tpl::JointState<NJOINTS, SCALAR>::Position> computeInverseKinematics(
        const tpl::RigidBodyPose<SCALAR> &eeBasePose,
        const std::vector<SCALAR> &freeJoints) const = 0;

    virtual std::vector<typename tpl::JointState<NJOINTS, SCALAR>::Position> computeInverseKinematics(
        const tpl::RigidBodyPose<SCALAR> &eeWorldPose,
        const tpl::RigidBodyPose<SCALAR> &baseWorldPose,
        const std::vector<SCALAR> &freeJoints) const = 0;
};

} /* namespace rbd */
} /* namespace ct */
