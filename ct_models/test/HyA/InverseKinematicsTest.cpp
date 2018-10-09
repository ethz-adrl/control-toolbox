/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include "ct/models/HyA/HyA.h"

#include <gtest/gtest.h>


const size_t njoints = ct::rbd::HyA::Kinematics::NJOINTS;

using HyAKinematicsAD_t = ct::rbd::HyA::tpl::Kinematics<ct::core::ADCGScalar>;
using HyAKinematics_t = ct::rbd::HyA::tpl::Kinematics<double>;



int main(int argc, char* argv[])
{
	size_t eeInd = 1;

	std::shared_ptr<ct::rbd::IKCostEvaluator<HyAKinematicsAD_t>> ikCostEvaluator (new ct::rbd::IKCostEvaluator<HyAKinematicsAD_t> (eeInd));

	return 1;
}
