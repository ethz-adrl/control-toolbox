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

#include "../../models/testhyq/RobCoGenTestHyQ.h"
#include <gtest/gtest.h>

using namespace ct;
using namespace rbd;

//TEST(TaskspaceCostFunctionTests, TestTaskSpacePositionTerm)
void testPosition()
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
        new TermTaskspacePosition<KinTpl_t, true, hyqStateDim, hyqControlDim>(eeId, Q));

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

    //! compare auto-diff against num-diff
    //    ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    //    ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());
}


//TEST(TaskspaceCostFunctionTests, TestTaskSpacePoseTerm)
void testPose()
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

    Adcf->stateDerivativeIntermediateTest();
    Adcf->controlDerivativeIntermediateTest();

    //! compare auto-diff against num-diff
    //    ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
    //    ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());
}
