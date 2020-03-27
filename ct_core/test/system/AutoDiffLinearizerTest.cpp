/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/core/core.h>
#include "TestNonlinearSystem.h"
#include "TestDiscreteNonlinearSystem.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace ct::core;
using std::shared_ptr;


/*!
 *  \example AutoDiffLinearizerTest.cpp
 *
 *  This unit test serves as example how to use the SystemLinearizer (numerical differentiation) and
 *  the Autodiff-Linearizer (automatic differentiation)
 */
TEST(AutoDiffLinearizerTest, SystemLinearizerComparison)
{
    // typedefs for the auto-differentiable system
    typedef CppAD::AD<double> AD_Scalar;
    typedef tpl::TestNonlinearSystem<double, CONTINUOUS_TIME> TestNonlinearSystemD;
    typedef tpl::TestNonlinearSystem<AD_Scalar, CONTINUOUS_TIME> TestNonlinearSystemAD;

    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystemD::STATE_DIM;
    const size_t control_dim = TestNonlinearSystemD::CONTROL_DIM;
    using State = EuclideanState<state_dim, double>;
    using AD_State = EuclideanState<state_dim, AD_Scalar>;

    // handy typedefs for the Jacobian
    typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
    typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    double w_n = 100;
    shared_ptr<TestNonlinearSystemD> nonlinearSystem(new TestNonlinearSystemD(w_n));
    shared_ptr<TestNonlinearSystemAD> nonlinearSystemAD(new TestNonlinearSystemAD(AD_Scalar(w_n)));

    // create a linearizer that applies numerical differentiation
    SystemLinearizer<State, control_dim, CONTINUOUS_TIME> systemLinearizer(nonlinearSystem);

    // create a linearizer that uses codegeneration
    AutoDiffLinearizer<State, AD_State, control_dim, CONTINUOUS_TIME> adLinearizer(nonlinearSystemAD);
    std::shared_ptr<AutoDiffLinearizer<State, AD_State, control_dim, CONTINUOUS_TIME>> adLinearizerClone(
        adLinearizer.clone());

    // create state, control and time variables
    State x;
    ControlVector<TestNonlinearSystemD::CONTROL_DIM> u;
    double t = 0;

    for (size_t i = 0; i < 1000; i++)
    {
        // set a random state
        x.setRandom();
        u.setRandom();

        // use the numerical differentiation linearizer
        A_type A_system = systemLinearizer.getDerivativeState(x, u, t);
        B_type B_system = systemLinearizer.getDerivativeControl(x, u, t);

        // use the auto differentiation linearzier
        A_type A_ad = adLinearizer.getDerivativeState(x, u, t);
        B_type B_ad = adLinearizer.getDerivativeControl(x, u, t);

        A_type A_adCloned = adLinearizerClone->getDerivativeState(x, u, t);
        B_type B_adCloned = adLinearizerClone->getDerivativeControl(x, u, t);

        // verify the result
        ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);

        ASSERT_LT((A_system - A_adCloned).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_adCloned).array().abs().maxCoeff(), 1e-5);
    }
}


TEST(AutoDiffDiscreteLinearizerTest, DiscreteSystemLinearizerComparison)
{
    // typedefs for the auto-differentiable system
    typedef CppAD::AD<double> AD_Scalar;
    typedef tpl::TestDiscreteNonlinearSystem<double> TestDiscreteNonlinearSystem;
    typedef tpl::TestDiscreteNonlinearSystem<AD_Scalar> TestDiscreteNonlinearSystemAD;

    // define the dimensions of the system
    const size_t state_dim = TestDiscreteNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestDiscreteNonlinearSystem::CONTROL_DIM;
    using State = EuclideanState<state_dim, double>;
    using AD_State = EuclideanState<state_dim, AD_Scalar>;

    // handy typedefs for the Jacobian
    typedef StateMatrix<state_dim, double> A_type;
    typedef StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double rate = 0.1;
    shared_ptr<TestDiscreteNonlinearSystem> nonlinearSystem(new TestDiscreteNonlinearSystem(rate));
    shared_ptr<TestDiscreteNonlinearSystemAD> nonlinearSystemAD(new TestDiscreteNonlinearSystemAD(AD_Scalar(rate)));

    // create a linearizer that applies numerical differentiation
    SystemLinearizer<State, control_dim, DISCRETE_TIME> systemLinearizer(nonlinearSystem);

    // create a linearizer that uses auto differentiation
    AutoDiffLinearizer<State, AD_State, control_dim, DISCRETE_TIME> adLinearizer(nonlinearSystemAD);
    std::shared_ptr<AutoDiffLinearizer<State, AD_State, control_dim, DISCRETE_TIME>> adLinearizerClone(
        adLinearizer.clone());

    // create state, control and time variables
    State x;
    ControlVector<TestDiscreteNonlinearSystem::CONTROL_DIM> u;
    int n = 0;

    for (size_t i = 0; i < 100; i++)
    {
        // set a random state
        x.setRandom();
        u.setRandom();

        // use the numerical differentiation linearizer
        A_type A_system = systemLinearizer.getDerivativeState(x, u, n);
        B_type B_system = systemLinearizer.getDerivativeControl(x, u, n);

        // analytic derivative
        A_type A_system_analytic = nonlinearSystem->get_A_analytic(x, u);
        B_type B_system_analytic = nonlinearSystem->get_B_analytic(x, u);

        ASSERT_LT((A_system - A_system_analytic).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_system_analytic).array().abs().maxCoeff(), 1e-5);

        // use the auto differentiation linearzier
        A_type A_ad = adLinearizer.getDerivativeState(x, u, n);
        B_type B_ad = adLinearizer.getDerivativeControl(x, u, n);

        A_type A_adCloned = adLinearizerClone->getDerivativeState(x, u, n);
        B_type B_adCloned = adLinearizerClone->getDerivativeControl(x, u, n);

        // verify the result

        ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);

        ASSERT_LT((A_system - A_adCloned).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_adCloned).array().abs().maxCoeff(), 1e-5);
    }
}


TEST(AutoDiffLinearizerTestMP, SystemLinearizerComparisonMP)
{
    // typedefs for the auto-differentiable system
    typedef CppAD::AD<double> AD_Scalar;
    typedef tpl::TestNonlinearSystem<double, CONTINUOUS_TIME> TestNonlinearSystemD;
    typedef tpl::TestNonlinearSystem<AD_Scalar, CONTINUOUS_TIME> TestNonlinearSystemAD;

    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystemD::STATE_DIM;
    const size_t control_dim = TestNonlinearSystemD::CONTROL_DIM;
    using State = EuclideanState<state_dim, double>;
    using AD_State = EuclideanState<state_dim, AD_Scalar>;

    typedef std::shared_ptr<AutoDiffLinearizer<State, AD_State, control_dim, CONTINUOUS_TIME>> AdLinearizerPtr;
    typedef ControlVector<control_dim> ControlVector;

    // handy typedefs for the Jacobian
    typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
    typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    double w_n = 100;
    shared_ptr<TestNonlinearSystemD> nonlinearSystem(new TestNonlinearSystemD(w_n));
    shared_ptr<TestNonlinearSystemAD> nonlinearSystemAD(new TestNonlinearSystemAD(AD_Scalar(w_n)));

    // create a linearizer that applies numerical differentiation
    SystemLinearizer<State, control_dim, CONTINUOUS_TIME> systemLinearizer(nonlinearSystem);
    AutoDiffLinearizer<State, AD_State, control_dim, CONTINUOUS_TIME> adLinearizer(nonlinearSystemAD);

    std::vector<std::shared_ptr<SystemLinearizer<State, control_dim, CONTINUOUS_TIME>>> systemLinearizers;

    size_t runs = 1000;
    size_t numThreads = 5;

    // The ad objects cannot yet be initialized here
    for (size_t i = 0; i < numThreads; ++i)
        systemLinearizers.push_back(
            std::shared_ptr<SystemLinearizer<State, control_dim, CONTINUOUS_TIME>>(systemLinearizer.clone()));

    // Count in the main thread
    CppadParallel::initParallel(numThreads + 1);
    for (size_t n = 0; n < runs; ++n)
    {
        std::vector<std::thread> threads;

        for (size_t i = 0; i < numThreads; ++i)
        {
            threads.push_back(std::thread([i, state_dim, control_dim, &adLinearizer, &systemLinearizers]() {
                // The ad objects are initialized here, because they need to be associated with the specfic thread number
                AdLinearizerPtr adLinearizerLocal = AdLinearizerPtr(adLinearizer.clone());

                State x;
                ControlVector u;
                double t = 0.0;

                x.setRandom();
                u.setRandom();

                // use the numerical differentiation linearizer
                A_type A_system = systemLinearizers[i]->getDerivativeState(x, u, t);
                B_type B_system = systemLinearizers[i]->getDerivativeControl(x, u, t);

                // use the auto differentiation linearzier
                A_type A_ad = adLinearizerLocal->getDerivativeState(x, u, t);
                B_type B_ad = adLinearizerLocal->getDerivativeControl(x, u, t);

                // verify the result
                ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
                ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);
            }));
        }

        for (auto& thr : threads)
            thr.join();
    }

    CppadParallel::resetParallel();
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
