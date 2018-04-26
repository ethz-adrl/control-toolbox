/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>
#include <ct/rbd/robot/jacobian/ConstraintJacobian.h>
#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;

TEST(JacobianTest, JacobianDerivativeTest)
{
    int NTESTS = 100;

    typedef double valType;
    typedef TestHyQ::tpl::Kinematics<valType> Kinematics_t;
    typedef tpl::ConstraintJacobian<Kinematics_t, 12, 12, valType> ConstraintJacobian_t;

    ConstraintJacobian_t Jc_t;
    for (int ee = 0; ee < 4; ee++)
    {
        Jc_t.ee_indices_.push_back(ee);
        Jc_t.eeInContact_[ee] = true;
    }

    ConstraintJacobian_t::jacobian_t dJdt1, dJdt2, diff;
    TestHyQ::tpl::Dynamics<valType>::RBDState_t hyq_state;

    for (int n = 0; n < NTESTS; n++)
    {
        hyq_state.setRandom();

        Jc_t.getJacobianOriginDerivative(hyq_state, dJdt1);
        Jc_t.getJacobianOriginDerivativeNumdiff(hyq_state, dJdt2);
        diff = dJdt1 - dJdt2;
        auto maxError = diff.cwiseAbs().maxCoeff();

        ASSERT_LT(maxError, 1e-3);
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
