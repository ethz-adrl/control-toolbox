/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <gtest/gtest.h>

#include <ct/optcon/optcon.h>

namespace ct {
namespace optcon {
namespace example {


/*!
 * this is simply a little integration test that shows that the tracking cost function term builds and that
 * its analytic derivatives match NumDiff derivatives.
 */
TEST(TrackingTest, TrackingTermTest)
{
    const size_t state_dim = 12;
    const size_t control_dim = 4;

    // analytical costfunction
    std::shared_ptr<CostFunctionAnalytical<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());

    Eigen::Matrix<double, state_dim, state_dim> Q;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q.setIdentity();
    R.setIdentity();

    // create a reference trajectory and fill it with random values
    core::StateTrajectory<state_dim> stateTraj;
    core::ControlTrajectory<control_dim> controlTraj;
    size_t trajSize = 50;
    bool timeIsAbsolute = true;
    for (size_t i = 0; i < trajSize; ++i)
    {
        stateTraj.push_back(core::StateVector<state_dim>::Random(), double(i), timeIsAbsolute);
        controlTraj.push_back(core::ControlVector<control_dim>::Random(), double(i), timeIsAbsolute);
    }

    std::shared_ptr<TermQuadTracking<state_dim, control_dim>> trackingTerm(new TermQuadTracking<state_dim, control_dim>(
        Q, R, core::InterpolationType::LIN, core::InterpolationType::ZOH, true));

    trackingTerm->setStateAndControlReference(stateTraj, controlTraj);
    costFunction->addIntermediateTerm(trackingTerm);

    ct::core::StateVector<state_dim> x;
    ct::core::ControlVector<control_dim> u;
    x.setRandom();
    u.setRandom();
    double t = 0.0;

    costFunction->setCurrentStateAndControl(x, u, t);

    ASSERT_TRUE(costFunction->stateDerivativeIntermediateTest());
    ASSERT_TRUE(costFunction->controlDerivativeIntermediateTest());
}

}  // namespace example
}  // namespace optcon
}  // namespace ct


/*!
 * This unit test tests the tracking cost function and the tracking cost function term.
 * \example TrackingTest.cpp
 */
int main(int argc, char** argv)
{
    using namespace ct::optcon::example;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
