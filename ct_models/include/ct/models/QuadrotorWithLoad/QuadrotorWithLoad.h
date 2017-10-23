/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#include <Eigen/Core>

#include <ct/rbd/rbd.h>

//#include <ct/rbd/robot/RobCoGenContainer.h>
//#include <ct/rbd/robot/Kinematics.h>
//#include <ct/rbd/robot/Dynamics.h>

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
