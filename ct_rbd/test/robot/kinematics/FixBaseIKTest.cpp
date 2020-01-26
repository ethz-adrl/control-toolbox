/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include "../../models/testIrb4600/RobCoGenTestIrb4600.h"

#include <gtest/gtest.h>


const size_t njoints = ct::rbd::TestIrb4600::Kinematics::NJOINTS;

using Kinematics_t = ct::rbd::TestIrb4600::tpl::Kinematics<double>;
using IKProblem = ct::rbd::IKNLP<Kinematics_t>;
using IKNLPSolver = ct::rbd::IKNLPSolverIpopt<IKProblem, Kinematics_t>;


TEST(FixBaseInverseKinematicsTest, DISABLED_NLPIKTest)
{
    size_t eeInd = 0;

    ct::rbd::JointState<njoints>::Position jointLowerLimit = ct::rbd::TestIrb4600::jointLowerLimit();
    ct::rbd::JointState<njoints>::Position jointUpperLimit = ct::rbd::TestIrb4600::jointUpperLimit();

    std::shared_ptr<ct::rbd::IKCostEvaluator<Kinematics_t>> ikCostEvaluator(
        new ct::rbd::IKCostEvaluator<Kinematics_t>(eeInd));

    std::shared_ptr<IKProblem> ik_problem(new IKProblem(ikCostEvaluator, jointLowerLimit, jointUpperLimit));


    ct::optcon::NlpSolverSettings nlpSolverSettings;
    nlpSolverSettings.solverType_ = ct::optcon::NlpSolverType::IPOPT;
    nlpSolverSettings.ipoptSettings_.derivativeTest_ = "second-order";  // check derivatives
    nlpSolverSettings.ipoptSettings_.hessian_approximation_ = "exact";  // option: "limited-memory"
    nlpSolverSettings.ipoptSettings_.printLevel_ = 0;                   // avoid output

    ct::rbd::InverseKinematicsSettings ikSettings;
    ikSettings.maxNumTrials_ = 100;
    ikSettings.randomizeInitialGuess_ = true;
    ikSettings.validationTol_ = 1e-6;

    IKNLPSolver ikSolver(ik_problem, nlpSolverSettings, ikSettings, eeInd);

    // set desired end effector pose
    ct::rbd::RigidBodyPose ee_pose_des;
    ee_pose_des.position()(0) = 1.0;
    ee_pose_des.position()(1) = 1.0;
    ee_pose_des.position()(2) = 1.0;
    ikCostEvaluator->setTargetPose(ee_pose_des);

    IKNLPSolver::JointPositionsVector_t solutions;
    bool accurateSolutionFound = ikSolver.computeInverseKinematics(solutions, ee_pose_des);

    ASSERT_TRUE(accurateSolutionFound);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
