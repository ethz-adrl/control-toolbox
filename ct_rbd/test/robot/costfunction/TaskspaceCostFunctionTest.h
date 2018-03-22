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

TEST(TaskspaceCostFunctionTests, TestTaskSpacePositionTerm)
{
    typedef ct::core::ADCGScalar size_type;
    typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;

    KinTpl_t kynTpl;
    size_t eeId = 1;

    const size_t hyqStateDim = 36;
    const size_t hyqControlDim = 12;

    Eigen::Matrix<double, 3, 3> Q;
    Q.setIdentity();

    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> Adcf(
        new optcon::CostFunctionAD<hyqStateDim, hyqControlDim>());
    std::shared_ptr<TermTaskspacePosition<KinTpl_t, true, hyqStateDim, hyqControlDim>> term1(
        new TermTaskspacePosition<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Q, Eigen::Vector3d::Random()));

    Adcf->addFinalADTerm(term1, true);
    Adcf->addIntermediateADTerm(term1, true);

    Adcf->initialize();

    Eigen::Matrix<double, hyqStateDim, 1> x;
    Eigen::Matrix<double, hyqControlDim, 1> u;
    x.setRandom();
    u.setRandom();

    double t = 1.0;

    Adcf->setCurrentStateAndControl(x, u, t);

    //    Adcf->stateDerivativeIntermediateTest();
    //    Adcf->controlDerivativeIntermediateTest();

    //! compare auto-diff against num-diff
    ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());

    // check cloning for this term/costfunction combination
    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> Adcf_cloned(Adcf->clone());
    x.setRandom();
    u.setRandom();
    t = 2.0;
    Adcf->setCurrentStateAndControl(x, u, t);
    Adcf_cloned->setCurrentStateAndControl(x, u, t);

    ct::optcon::example::compareCostFunctionOutput(*Adcf_cloned, *Adcf);
}


TEST(TaskspaceCostFunctionTests, TestTaskSpacePoseTerm)
{
    typedef ct::core::ADCGScalar size_type;
    typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;

    KinTpl_t kynTpl;
    size_t eeId = 1;

    const size_t hyqStateDim = 36;
    const size_t hyqControlDim = 12;

    // specify a penalty on the position error
    Eigen::Matrix<double, 3, 3> Qpos;
    Qpos.setIdentity();

    // specify a penalty on the orientation error
    double Qrot = 1.0;

    // specify a desired ee position
    Eigen::Matrix<double, 3, 1> w_pos_ee;
    w_pos_ee.setRandom();

    // specify a desired ee orientation as quaternion, to be used in first constructor
    Eigen::Quaternion<double> w_q_ee;
    w_q_ee.setIdentity();

    // get euler angles corresponding to the quaternion, to be used in second constructor
    Eigen::Matrix<double, 3, 1> w_euler_ee = w_q_ee.toRotationMatrix().eulerAngles(0, 1, 2);

    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> Adcf(
        new optcon::CostFunctionAD<hyqStateDim, hyqControlDim>());

    // test constructor taking the quaternion
    std::shared_ptr<TermTaskspacePose<KinTpl_t, true, hyqStateDim, hyqControlDim>> term1(
        new TermTaskspacePose<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Qpos, Qrot, w_pos_ee, w_q_ee));

    // test constructor using euler angles
    std::shared_ptr<TermTaskspacePose<KinTpl_t, true, hyqStateDim, hyqControlDim>> term2(
        new TermTaskspacePose<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Qpos, Qrot, w_pos_ee, w_euler_ee));

    Adcf->addFinalADTerm(term1, true);
    Adcf->addIntermediateADTerm(term1, true);

    Adcf->initialize();

    Eigen::Matrix<double, hyqStateDim, 1> x;
    Eigen::Matrix<double, hyqControlDim, 1> u;
    x.setRandom();
    u.setRandom();

    double t = 1.0;

    Adcf->setCurrentStateAndControl(x, u, t);

    //    Adcf->stateDerivativeIntermediateTest();
    //    Adcf->controlDerivativeIntermediateTest();

    //! compare auto-diff against num-diff
    ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());

    // check cloning for this term/costfunction combination
    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> Adcf_cloned(Adcf->clone());
    x.setRandom();
    u.setRandom();
    t = 2.0;
    Adcf->setCurrentStateAndControl(x, u, t);
    Adcf_cloned->setCurrentStateAndControl(x, u, t);
    ct::optcon::example::compareCostFunctionOutput(*Adcf_cloned, *Adcf);
}
