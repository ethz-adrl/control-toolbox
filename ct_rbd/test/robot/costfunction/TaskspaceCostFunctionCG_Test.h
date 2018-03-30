/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include "../../models/testhyq/RobCoGenTestHyQ.h"
#include <gtest/gtest.h>
#include "../../../ct_optcon/test/costfunction/compareCostFunctions.h"

using namespace ct;
using namespace rbd;

// global vars
const size_t eeId = 1;
const size_t hyqStateDim = 36;
const size_t hyqControlDim = 12;

// specify a penalty on the position error
const Eigen::Matrix<double, 3, 3> Qpos = Eigen::Matrix<double, 3, 3>::Identity();

// specify a penalty on the orientation error
const double Qrot = 1.0;


//TEST(TaskspaceCostFunctionTests, TestTaskSpacePoseTerm)
void test()
{
    typedef ct::core::ADCGScalar size_type;
    typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;

    // specify a desired ee position
    Eigen::Matrix<double, 3, 1> w_pos_ee;
    w_pos_ee.setRandom();

    // specify a desired ee orientation as quaternion, to be used in first constructor
    Eigen::Quaternion<double> w_q_ee;
    w_q_ee.setIdentity();

    // get euler angles corresponding to the quaternion, to be used in second constructor
    Eigen::Matrix<double, 3, 1> w_euler_ee = w_q_ee.toRotationMatrix().eulerAngles(0, 1, 2);

    std::shared_ptr<optcon::CostFunctionAnalytical<hyqStateDim, hyqControlDim>> costFun(
        new optcon::CostFunctionAnalytical<hyqStateDim, hyqControlDim>());

    // test constructor taking the quaternion
    std::shared_ptr<TermTaskspacePoseCG<KinTpl_t, true, hyqStateDim, hyqControlDim>> term1(
        new TermTaskspacePoseCG<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Qpos, Qrot, w_pos_ee, w_q_ee));

    costFun->addIntermediateTerm(term1, true);
    costFun->initialize();

    Eigen::Matrix<double, hyqStateDim, 1> x;
    Eigen::Matrix<double, hyqControlDim, 1> u;
    x.setRandom();
    u.setRandom();

    double t = 1.0;

    costFun->setCurrentStateAndControl(x, u, t);


    std::cout << "eval int " << costFun->evaluateIntermediate() << std::endl;
    std::cout << "stateDerivativeIntermediate" << std::endl << costFun->stateDerivativeIntermediate().transpose() << std::endl;

    std::cout << "now changing ref position" << std::endl;
    w_pos_ee(0) += 1.0;
    w_pos_ee(2) += 1.0;
    term1->setReferencePosition(w_pos_ee);
    std::cout << "eval int " << costFun->evaluateIntermediate() << std::endl;
    std::cout << "stateDerivativeIntermediate" << std::endl << costFun->stateDerivativeIntermediate().transpose() << std::endl;

//    std::cout << "stateSecondDerivativeIntermediate " << std::endl
//              << costFun->stateSecondDerivativeIntermediate() << std::endl;


//    costFun->stateDerivativeIntermediateTest();
    //    costFun->controlDerivativeIntermediateTest();

    //    //! compare auto-diff against num-diff
    //    ASSERT_TRUE(costFun->stateDerivativeIntermediateTest());
    //    ASSERT_TRUE(costFun->controlDerivativeIntermediateTest());
    //
    //    // check cloning for this term/costfunction combination
    //    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> costFun_cloned(costFun->clone());
    //    x.setRandom();
    //    u.setRandom();
    //    t = 2.0;
    //    costFun->setCurrentStateAndControl(x, u, t);
    //    costFun_cloned->setCurrentStateAndControl(x, u, t);
    //    ct::optcon::example::compareCostFunctionOutput(*costFun_cloned, *costFun);
}
