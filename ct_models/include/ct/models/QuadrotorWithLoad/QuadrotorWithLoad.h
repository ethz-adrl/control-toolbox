/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/rbd.h>

#include "generated/declarations.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/jacobians.h"
#include "generated/jsim.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"
#include "generated/traits.h"


// define namespace and base
#define ROBCOGEN_NS ct_quadrotor
#define TARGET_NS ct_quadrotor

// define the links
#define CT_BASE fr_body
#define CT_L0 link1
#define CT_L1 link2

// define first end effector, the endeffector frame
#define CT_N_EE 1
#define CT_EE0 fr_ee
#define CT_EE0_IS_ON_LINK 1
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 1

/*
namespace ct {
namespace rbd {

typedef RobCoGenContainer<
		iit::ct_quadrotor::Traits,
		iit::ct_quadrotor::LinkDataMap
		>
RobCoGenQuadrotorWithLoad;

const size_t numEE = 1;

typedef Kinematics<RobCoGenQuadrotorWithLoad, numEE> KinematicsQuadrotorWithLoad;
typedef Dynamics<RobCoGenQuadrotorWithLoad, numEE, true> DynamicsQuadrotorWithLoad;

static const size_t nStates = 12;
static const size_t nControls = 4;
static const size_t nJoints = 2;

template <>
inline typename RobCoGenQuadrotorWithLoad::HomogeneousTransform RobCoGenQuadrotorWithLoad::getHomogeneousTransformBaseLinkById(
		size_t linkId, const JointState<NJOINTS>::Position& jointPosition)
{
	return ct_quadrotor::getTransformBaseLinkById(homogeneousTransforms(), linkId, jointPosition);
}

template <>
inline typename RobCoGenQuadrotorWithLoad::ForceTransform RobCoGenQuadrotorWithLoad::getForceTransformLinkBaseById(
		size_t linkId, const JointState<NJOINTS>::Position& jointPosition)
{
	return ct_quadrotor::getTransformLinkBaseById(forceTransforms(), linkId, jointPosition);
}

template <>
inline typename RobCoGenQuadrotorWithLoad::HomogeneousTransform RobCoGenQuadrotorWithLoad::getHomogeneousTransformBaseEEById(
		size_t eeId, const JointState<NJOINTS>::Position& jointPosition)
{
	return ct_quadrotor::getTransformBaseEEById(homogeneousTransforms(), eeId, jointPosition);
}

template <>
inline typename RobCoGenQuadrotorWithLoad::Jacobian RobCoGenQuadrotorWithLoad::getJacobianBaseEEbyId(
		size_t eeId, const JointState<NJOINTS>::Position& jointPosition)
{
	return ct_quadrotor::getJacobianBaseEEbyId<Jacobians, NJOINTS>(jacobians(), eeId, jointPosition);
}



template <>
inline void KinematicsQuadrotorWithLoad::initEndeffectors(std::array<EndEffector<NJOINTS>, numEE>& endeffectors)
{
	for (size_t i=0; i<numEE; i++)
	{
		endeffectors[i].setLinkId(ct_quadrotor::eeIdToLinkId(i));
	}
}

}
}*/

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
