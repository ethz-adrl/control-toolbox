/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"
#include <gtest/gtest.h>

using namespace ct;
using namespace rbd;

//TEST(TaskspaceCostFunctionTests, TaskspaceCostFunctionTest)
void test()
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
    std::shared_ptr<TermTaskspace<KinTpl_t, true, hyqStateDim, hyqControlDim>> term1(
        new TermTaskspace<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Q));

    Adcf->addFinalADTerm(term1, true);
    Adcf->addIntermediateADTerm(term1, true);

    Adcf->initialize();

    Eigen::Matrix<double, hyqStateDim, 1> x;
    Eigen::Matrix<double, hyqControlDim, 1> u;
    x.setRandom();
    u.setRandom();

    double t = 1.0;

    Adcf->setCurrentStateAndControl(x, u, t);

    Adcf->stateDerivativeIntermediateTest();
    Adcf->controlDerivativeIntermediateTest();

    //	ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    //	ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());
}


int main(int argc, char **argv)
{
    //  testing::InitGoogleTest(&argc, argv);
    test();
    return true;
    //  return RUN_ALL_TESTS();
}
