/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct;
using namespace rbd;
using namespace optcon;

/*!
 * @brief This method is called from different unit tests in order to compare the cost, first and second order gradients of two cost functions
 * @param costFunction the first cost function to be compared
 * @param costFunction2 the second cost function to be compared
 */
template <size_t state_dim, size_t control_dim>
void compareCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction,
    CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
    ASSERT_NEAR(costFunction.evaluateIntermediate(), costFunction2.evaluateIntermediate(), 1e-9);
    ASSERT_NEAR(costFunction.evaluateTerminal(), costFunction2.evaluateTerminal(), 1e-9);

    ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(costFunction2.stateDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateDerivativeTerminal().isApprox(costFunction2.stateDerivativeTerminal()));

    ASSERT_TRUE(
        costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction2.stateSecondDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateSecondDerivativeTerminal().isApprox(costFunction2.stateSecondDerivativeTerminal()));

    ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(costFunction2.controlDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.controlDerivativeTerminal().isApprox(costFunction2.controlDerivativeTerminal()));

    ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(
        costFunction2.controlSecondDerivativeIntermediate()));
    ASSERT_TRUE(
        costFunction.controlSecondDerivativeTerminal().isApprox(costFunction2.controlSecondDerivativeTerminal()));

    ASSERT_TRUE(
        costFunction.stateControlDerivativeIntermediate().isApprox(costFunction2.stateControlDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateControlDerivativeTerminal().isApprox(costFunction2.stateControlDerivativeTerminal()));

    // second derivatives have to be symmetric
    ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(
        costFunction.stateSecondDerivativeIntermediate().transpose()));
    ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(
        costFunction.controlSecondDerivativeIntermediate().transpose()));
}


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

    // compare auto-diff against num-diff
    ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());

    // check cloning for this term/costfunction combination
    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> Adcf_cloned(Adcf->clone());
    x.setRandom();
    u.setRandom();
    t = 2.0;
    Adcf->setCurrentStateAndControl(x, u, t);
    Adcf_cloned->setCurrentStateAndControl(x, u, t);

    compareCostFunctionOutput(*Adcf_cloned, *Adcf);
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

    // compare auto-diff against num-diff
    ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());

    // check cloning for this term/costfunction combination
    std::shared_ptr<optcon::CostFunctionAD<hyqStateDim, hyqControlDim>> Adcf_cloned(Adcf->clone());
    x.setRandom();
    u.setRandom();
    t = 2.0;
    Adcf->setCurrentStateAndControl(x, u, t);
    Adcf_cloned->setCurrentStateAndControl(x, u, t);
    compareCostFunctionOutput(*Adcf_cloned, *Adcf);
}


TEST(TaskspaceCostFunctionTests, TestTaskSpacePoseTermCG)
{
    typedef ct::core::ADCGScalar size_type;
    typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;

    // global vars
    const size_t eeId = 1;
    const size_t hyqStateDim = 36;
    const size_t hyqControlDim = 12;

    // specify a penalty on the position error
    const Eigen::Matrix<double, 3, 3> Qpos = Eigen::Matrix<double, 3, 3>::Identity();

    // specify a penalty on the orientation error
    const double Qrot = 1.0;

    // specify a desired ee position
    Eigen::Matrix<double, 3, 1> w_pos_ee;
    w_pos_ee.setRandom();

    // specify a desired ee orientation as quaternion, to be used in first constructor
    Eigen::Quaternion<double> w_q_ee;
    w_q_ee.setIdentity();

    // test constructor taking the quaternion
    std::shared_ptr<TermTaskspacePoseCG<KinTpl_t, true, hyqStateDim, hyqControlDim>> term1(
        new TermTaskspacePoseCG<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Qpos, Qrot, w_pos_ee, w_q_ee));

    // create and initialize the cost function
    std::shared_ptr<optcon::CostFunctionAnalytical<hyqStateDim, hyqControlDim>> costFun(
        new optcon::CostFunctionAnalytical<hyqStateDim, hyqControlDim>());

    costFun->addIntermediateTerm(term1);
    costFun->initialize();

    Eigen::Matrix<double, hyqStateDim, 1> x;
    Eigen::Matrix<double, hyqControlDim, 1> u;
    x.setRandom();
    u.setRandom();

    double t = 1.0;

    // TEST 1: compare auto-diff against num-diff
    // ==========================================
    ASSERT_TRUE(costFun->stateDerivativeIntermediateTest());
    ASSERT_TRUE(costFun->controlDerivativeIntermediateTest());

    // TEST 2: cost must not be the same after the reference position and orientation got changed
    // ==========================================
    costFun->setCurrentStateAndControl(x, u, t);
    double cost_orig = costFun->evaluateIntermediate();
    Eigen::VectorXd stateDerivative_orig = costFun->stateDerivativeIntermediate();
    // change reference parameters
    w_pos_ee.setRandom();
    Eigen::Quaterniond testq = w_q_ee * Eigen::Quaterniond(0, 1, 0, 0);
    term1->setReferencePosition(w_pos_ee);
    term1->setReferenceOrientation(testq);
    // new evaluation
    double cost_new = costFun->evaluateIntermediate();
    Eigen::VectorXd stateDerivative_new = costFun->stateDerivativeIntermediate();
    ASSERT_TRUE(cost_orig - cost_new > 1e-8);
    ASSERT_TRUE((stateDerivative_orig - stateDerivative_new).array().maxCoeff() > 1e-8);

    // TEST 3: check that the reference position and orientation got updated correctly
    // ==========================================
    Eigen::Vector3d w_pos_ee_retrieved = term1->getReferencePosition();
    ASSERT_TRUE((w_pos_ee - w_pos_ee_retrieved).array().maxCoeff() < 1e-6);
    Eigen::Matrix3d matOriginal = testq.toRotationMatrix();
    Eigen::Matrix3d matRetrieved = term1->getReferenceOrientation().toRotationMatrix();
    ASSERT_TRUE((matOriginal - matRetrieved).array().maxCoeff() < 1e-6);

    // TEST 4: check cloning for this term/costfunction combination
    // ==========================================
    std::shared_ptr<optcon::CostFunctionAnalytical<hyqStateDim, hyqControlDim>> costFun_cloned(costFun->clone());
    x.setRandom();
    u.setRandom();
    t = 2.0;
    costFun->setCurrentStateAndControl(x, u, t);
    costFun_cloned->setCurrentStateAndControl(x, u, t);
    compareCostFunctionOutput(*costFun_cloned, *costFun);
}
