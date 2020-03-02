/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

//#include <manif/manif.h>
//
//#include <ct/core/types/ManifoldState.h>
//#include <ct/core/types/ManifoldState-impl.h>
//#include <ct/core/types/EuclideanState.h>
//
//#include <ct/core/integration/Observer.h>
//#include <ct/core/integration/Observer-impl.h>
//#include <ct/core/integration/Integrator.h>
//#include <ct/core/integration/Integrator-impl.h>
//
//#include <ct/core/systems/continuous_time/ControlledSystem.h>
//#include <ct/core/systems/continuous_time/ControlledSystem-impl.h>
//
//#include <ct/core/control/continuous_time/ConstantController.h>
//#include <ct/core/control/continuous_time/ConstantController-impl.h>
//
//#include <ct/core/systems/continuous_time/System-impl.h>
//#include <ct/core/types/TypeTraits.h>

#include <ct/core/core.h>


const bool verbose = false;

using namespace ct::core;
using namespace manif;

using Manifold = ManifoldState<SE3d, SE3Tangentd>;
const size_t control_dim = 6;

class TestSystem : public ControlledSystem<Manifold, control_dim>
{
public:
    virtual TestSystem* clone() const override { return new TestSystem(); }
    void computeControlledDynamics(const Manifold& state,
        const double& t,
        const ControlVector<control_dim>& control,
        typename Manifold::Tangent& derivative) override
    {
        derivative = control;  // todo
    }
};

TEST(ManifoldTest, EuclideanTraitsTest)
{
    using namespace ct::core;

    static_assert(is_euclidean<Manifold>::value == false, "");
    static_assert(is_euclidean<double>::value == false, "");
    static_assert(is_euclidean<std::string>::value == false, "");

    static_assert(is_euclidean<EuclideanState<2>>::value == true, "");
    static_assert(is_euclidean<EuclideanState<control_dim, double>>::value == true, "");

    static_assert(is_real_euclidean<EuclideanState<2>>::value, "");
    static_assert(is_real_euclidean<EuclideanState<control_dim, double>>::value == true, "");

    static_assert(is_real_euclidean<EuclideanState<2>>::value, "");
    static_assert(is_real_euclidean<EuclideanState<control_dim, float>>::value == true, "");

    static_assert(is_real_euclidean<EuclideanState<control_dim, int>>::value == false, "");
    static_assert(is_real_euclidean<Manifold>::value == false, "");

    ASSERT_TRUE(true);
}


TEST(ManifoldIntegrationTest, integrate_n_steps_comparison)
{
    const double dt = 0.001;
    // define integration times
    Time startTime = 0.0;
    size_t nsteps = 1.0 / dt;

    std::shared_ptr<ConstantController<Manifold, control_dim>> ctrl(new ConstantController<Manifold, control_dim>());
    const auto randomControl = ControlVector<control_dim>::Random();
    ctrl->setControl(randomControl);

    std::shared_ptr<TestSystem> testSystem(new TestSystem());
    testSystem->setController(ctrl);

    using Integrator_t = Integrator<Manifold>;

    try
    {
        Integrator_t integrator_euler(testSystem, EULER);
        Integrator_t integrator_rk4(testSystem, RK4);
        Integrator_t integrator_euler_ct(testSystem, EULERCT);
        Integrator_t integrator_rk4_ct(testSystem, RK4CT);
        // create an initial state
        Manifold initialState_euler_ct, initialState_rk4_ct, initialState_euler, initialState_rk4;

        {
            initialState_euler_ct.setIdentity();
            initialState_rk4_ct.setIdentity();
            initialState_euler.setIdentity();
            initialState_rk4.setIdentity();

            integrator_euler.integrate_n_steps(initialState_euler, startTime, nsteps, dt);
            integrator_rk4.integrate_n_steps(initialState_rk4, startTime, nsteps, dt);
            integrator_euler_ct.integrate_n_steps(initialState_euler_ct, startTime, nsteps, dt);
            integrator_rk4_ct.integrate_n_steps(initialState_rk4_ct, startTime, nsteps, dt);

            // compare different stepper results with 'integrate_n_steps'
            ASSERT_TRUE(initialState_euler_ct.isApprox(initialState_euler));
            ASSERT_TRUE(initialState_euler_ct.isApprox(initialState_rk4));
            ASSERT_TRUE(initialState_euler_ct.isApprox(initialState_rk4_ct));
        }

        {
            initialState_euler_ct.setIdentity();
            initialState_rk4_ct.setIdentity();
            initialState_euler.setIdentity();
            initialState_rk4.setIdentity();

            // create containers
            DiscreteArray<Manifold> stateTrajectory_euler, stateTrajectory_rk4, stateTrajectory_euler_ct,
                stateTrajectory_rk4_ct;
            TimeArray timeTrajectory_euler, timeTrajectory_rk4, timeTrajectory_euler_ct, timeTrajectory_rk4_ct;

            // integrate boost and ct-steppers with integrate_n_steps
            integrator_euler.integrate_n_steps(
                initialState_euler, startTime, nsteps, dt, stateTrajectory_euler, timeTrajectory_euler);
            integrator_euler_ct.integrate_n_steps(
                initialState_euler_ct, startTime, nsteps, dt, stateTrajectory_euler_ct, timeTrajectory_euler_ct);
            integrator_rk4.integrate_n_steps(
                initialState_rk4, startTime, nsteps, dt, stateTrajectory_rk4, timeTrajectory_rk4);
            integrator_rk4_ct.integrate_n_steps(
                initialState_rk4_ct, startTime, nsteps, dt, stateTrajectory_rk4_ct, timeTrajectory_rk4_ct);

            if (verbose)
            {  // print trajectory to cout
                for (size_t i = 0; i < stateTrajectory_euler.size(); i++)
                    std::cout << stateTrajectory_euler[i] << std::endl;
            }

            ASSERT_TRUE(stateTrajectory_euler.back().isApprox(stateTrajectory_euler_ct.back()));
            ASSERT_TRUE(stateTrajectory_euler.back().isApprox(stateTrajectory_rk4.back()));
            ASSERT_TRUE(stateTrajectory_euler.back().isApprox(stateTrajectory_rk4_ct.back()));
        }

    } catch (std::exception& e)
    {
        std::cout << "Caught exception." << std::endl;
        std::cout << e.what() << std::endl;
        FAIL();
    }
}


TEST(ManifoldIntegrationTest, stepperTests)
{
    using Integrator_t = Integrator<Manifold>;
    using IntegratorPtr_t = std::shared_ptr<Integrator_t>;

    try
    {
        std::shared_ptr<ConstantController<Manifold, control_dim>> ctrl(
            new ConstantController<Manifold, control_dim>());
        const auto randomControl = ControlVector<control_dim>::Random();
        ctrl->setControl(randomControl);

        std::shared_ptr<TestSystem> testSystem(new TestSystem());
        testSystem->setController(ctrl);

        std::vector<IntegratorPtr_t> integrators;

        double dt = 0.0001;

        // define integration times
        Time startTime = 0.0;
        Time finalTime = startTime + 1.0;
        size_t nsteps = 10000;

        // create an initial state
        Manifold initialState;
        initialState.setIdentity();

        integrators.clear();

        integrators.push_back(IntegratorPtr_t(new Integrator_t(testSystem, EULER)));
        integrators.push_back(IntegratorPtr_t(new Integrator_t(testSystem, RK4)));
        integrators.push_back(IntegratorPtr_t(new Integrator_t(testSystem, RK78)));
        integrators.push_back(IntegratorPtr_t(new Integrator_t(testSystem, MODIFIED_MIDPOINT)));

        // create trajectory containers
        size_t integratorFunctions = 4;  // integrate_const, integrate_n_steps, ...
        size_t nIntegrators = integrators.size();
        size_t nResults = nIntegrators * integratorFunctions;

        std::vector<DiscreteArray<Manifold>> stateTrajectories(nResults);
        std::vector<TimeArray> timeTrajectories(nResults);
        std::vector<Manifold> finalStates(nResults);

        Manifold initialStateLocal;
        for (size_t j = 0; j < nIntegrators; j++)
        {
            initialStateLocal = initialState;
            integrators[j]->integrate_const(initialStateLocal, startTime, finalTime, dt,
                stateTrajectories[j * integratorFunctions + 0], timeTrajectories[j * integratorFunctions + 0]);

            initialStateLocal = initialState;
            integrators[j]->integrate_n_steps(initialStateLocal, startTime, nsteps, dt,
                stateTrajectories[j * integratorFunctions + 1], timeTrajectories[j * integratorFunctions + 1]);

            initialStateLocal = initialState;
            integrators[j]->integrate_adaptive(initialStateLocal, startTime, finalTime,
                stateTrajectories[j * integratorFunctions + 2], timeTrajectories[j * integratorFunctions + 2], dt);

            initialStateLocal = initialState;
            for (size_t k = 0; k < nsteps + 1; k++)
            {
                timeTrajectories[j * integratorFunctions + 3].push_back(k * dt);
            }
            integrators[j]->integrate_times(initialStateLocal, timeTrajectories[j * integratorFunctions + 3],
                stateTrajectories[j * integratorFunctions + 3], dt);
        }

        // check the results
        for (size_t j = 0; j < nResults; j++)
        {
            // we should get at least two points, start and end
            ASSERT_GT(stateTrajectories[j].size(), 2);
            ASSERT_GT(timeTrajectories[j].size(), 2);

            // we should get equal number of states and times
            ASSERT_TRUE(stateTrajectories[j].front().isApprox(initialState));
            ASSERT_EQ(stateTrajectories[j].size(), timeTrajectories[j].size());

            // start and end should be correct
            ASSERT_NEAR(timeTrajectories[j].front(), startTime, dt);
            ASSERT_NEAR(timeTrajectories[j].back(), finalTime, dt);

            // check ordering of time stamps
            for (size_t k = 1; k < timeTrajectories[j].size(); k++)
            {
                ASSERT_GT(timeTrajectories[j][k], timeTrajectories[j][k - 1]);

                if (j % 4 == 0 || j % 4 == 1)
                {
                    // check equidistance
                    ASSERT_NEAR(timeTrajectories[j][k] - timeTrajectories[j][k - 1], dt, 1e-6);
                }
            }

            if (j > 1)
            {
                ASSERT_TRUE(stateTrajectories[j].back().isApprox(stateTrajectories[j - 1].back()));
            }
        }

    } catch (const std::exception& e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        FAIL();
    }
}
