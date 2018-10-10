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
    size_t eeInd = 0;

    ct::rbd::JointState<njoints>::Position jointLowerLimit = ct::models::HyA::jointLowerLimit();
    ct::rbd::JointState<njoints>::Position jointUpperLimit = ct::models::HyA::jointUpperLimit();

    std::shared_ptr<ct::rbd::IKCostEvaluator<HyAKinematicsAD_t>> ikCostEvaluator(
        new ct::rbd::IKCostEvaluator<HyAKinematicsAD_t>(eeInd));

    std::shared_ptr<ct::rbd::IKNLP<HyAKinematicsAD_t>> ik_problem(
        new ct::rbd::IKNLP<HyAKinematicsAD_t>(ikCostEvaluator, jointLowerLimit, jointUpperLimit));


    ct::optcon::NlpSolverSettings nlpSolverSettings;
    nlpSolverSettings.solverType_ = ct::optcon::NlpSolverType::IPOPT;
    nlpSolverSettings.ipoptSettings_.derivativeTest_ = "first-order";
    nlpSolverSettings.ipoptSettings_.hessian_approximation_ = "limited-memory";

    std::shared_ptr<ct::optcon::IpoptSolver> nlpSolver(new ct::optcon::IpoptSolver(ik_problem, nlpSolverSettings));

    if (!nlpSolver->isInitialized())
        nlpSolver->configure(nlpSolverSettings);

    nlpSolver->solve();


    return 1;
}
