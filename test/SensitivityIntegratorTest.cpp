/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <cmath>
#include <memory>

#include <ct/core/core.h>

// Bring in gtest
#include <gtest/gtest.h>


using namespace ct::core;
using std::shared_ptr;

const size_t stateSize = 2;
const size_t controlSize = 1;


class TestOscillator : public ControlledSystem<2, 1>
{
public:
    virtual TestOscillator* clone() const override { return new TestOscillator(*this); }
    virtual void computeControlledDynamics(const StateVector<2>& state,
        const double& t,
        const ControlVector<1>& control,
        StateVector<2>& derivative)
    {
        derivative(0) = state(1);
        derivative(1) = control(0) - 10 * state(0);  // mass is 1 kg
    }
};


//! Linear system class for the GNMS unit test
class LinearizedOscillator : public LinearSystem<2, 1>
{
public:
    state_matrix_t A_;
    state_control_matrix_t B_;


    const state_matrix_t& getDerivativeState(const StateVector<2>& x,
        const ControlVector<1>& u,
        const double t = 0.0) override
    {
        A_ << 0, 1, -10, 0;
        return A_;
    }

    const state_control_matrix_t& getDerivativeControl(const StateVector<2>& x,
        const ControlVector<1>& u,
        const double t = 0.0) override
    {
        B_ << 0, 1;
        return B_;
    }

    LinearizedOscillator* clone() const override { return new LinearizedOscillator(); };
};


//TEST(IntegrationTest, derivativeTest)
void test()
{
    try
    {
        double dt = 0.001;

        // define integration times
        Time startTime = 0.0;
        Time finalTime = startTime + 1.0;
        size_t nsteps = 10;  //1.0/dt;

        // create an initial state
        StateVector<stateSize> initialState;
        initialState << 1.0, 0.0;

        // create a 10 Hz second order system with damping 0.1
        double w_n = 3.14;
        double zeta = 0.1;

        shared_ptr<TestOscillator> oscillator(new TestOscillator);
        shared_ptr<LinearizedOscillator> oscillatorLinearized(new LinearizedOscillator);


        std::shared_ptr<ConstantController<2, 1>> controller(new ConstantController<2, 1>);
        oscillator->setController(controller);

        ControlVector<1> control;
        control.setConstant(1.0);
        controller->setControl(control);

        // create a 2 state integrator
        std::shared_ptr<SimpleSensitivityIntegratorCT<stateSize, controlSize>> integrator(
            new SimpleSensitivityIntegratorCT<stateSize, controlSize>(oscillator, ct::core::IntegrationType::EULERCT));

        integrator->setLinearSystem(oscillatorLinearized);


        StateVectorArray<stateSize> stateTrajectory;
        TimeArray timeTrajectory;
        StateVector<stateSize> finalState;

        std::cout << "Testing integration" << std::endl;

        std::cout << "init state is " << initialState.transpose() << std::endl;

        integrator->integrate_n_steps(initialState, startTime, nsteps, dt);

        std::cout << "resulting state is " << initialState.transpose() << std::endl;

        StateMatrix<2> A, Adiscretizer;
        StateControlMatrix<2, 1> B, Bdiscretizer;

        integrator->linearize();

        integrator->integrateSensitivityDX0(A, startTime, nsteps, dt);

        integrator->integrateSensitivityDU0(B, startTime, nsteps, dt);


        std::cout << "A from sens integrator" << std::endl << A << std::endl;
        std::cout << "B from sens integrator" << std::endl << B << std::endl;


        // compare to linear system discretizer discretizer

        LinearSystemDiscretizer<2, 1> discretizer(
            dt, oscillatorLinearized, LinearSystemDiscretizerSettings::APPROXIMATION::FORWARD_EULER);

        discretizer.getAandB(initialState, control, 0, Adiscretizer, Bdiscretizer);


        std::cout << "A from discretizer" << std::endl << Adiscretizer << std::endl;
        std::cout << "B from discretizer" << std::endl << Bdiscretizer << std::endl;

        //            for (size_t j=0; j<nResults; j++)
        //            {
        //                //std::cout << "Testing result number " << j << std::endl;
        //
        //                // we should get at least two points, start and end
        //                ASSERT_GT(stateTrajectories[j].size(), 2);
        //                ASSERT_GT(timeTrajectories[j].size(), 2);
        //
        //                // we should get equal number of states and times
        //                ASSERT_LT((stateTrajectories[j].front()-initialState).array().abs().maxCoeff(), 1e-6);
        //                ASSERT_EQ(stateTrajectories[j].size(), timeTrajectories[j].size());
        //
        //                // start and end should be correct
        //                ASSERT_NEAR(timeTrajectories[j].front(), startTime, dt);
        //                ASSERT_NEAR(timeTrajectories[j].back(), finalTime, dt);
        //
        //                // check ordering of time stamps
        //                for (size_t k=1; k<timeTrajectories[j].size(); k++)
        //                {
        //                    ASSERT_GT(timeTrajectories[j][k], timeTrajectories[j][k-1]);
        //
        //                    if(j%4 == 0 || j%4 == 1)
        //                    {
        //                        // check equidistance
        //                        ASSERT_NEAR(timeTrajectories[j][k] - timeTrajectories[j][k-1], dt, 1e-6);
        //                    }
        //                }
        //
        //                // check correctness, only valid for non-adaptive
        //                if (j%4 - 2 != 0)
        //                {
        //                    for (size_t k=1; k<stateTrajectories[j].size(); k++)
        //                    {
        //                        double derivativeNumDiff = stateTrajectories[j][k](0) - stateTrajectories[j][k-1](0);
        //                        double derivativeAnalytical = solution(timeTrajectories[j][k]) - solution(timeTrajectories[j][k-1]);
        //
        //                        ASSERT_NEAR(derivativeNumDiff, derivativeAnalytical, 1e-2);
        //                    }
        //                }
        //
        //                if(j>1)
        //                {
        //                    ASSERT_NEAR(stateTrajectories[j].back()(0), stateTrajectories[j-1].back()(0), 5e-2);
        //                }
        //            }

    } catch (...)
    {
        std::cout << "Caught exception." << std::endl;
        //		FAIL();
    }
}


/*!
 *  \example IntegrationTest.cpp
 *
 *  This unit test serves as example how to use different steppers from boost odeint for numerical integration.
 */
int main(int argc, char** argv)
{
    //  testing::InitGoogleTest(&argc, argv);
    //  return RUN_ALL_TESTS();
    test();
    return 1;
}
