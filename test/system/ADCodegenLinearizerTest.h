/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TestNonlinearSystem.h"

/*!
 * Just-in-time compilation test : compile cloned instance
 */
TEST(ADCodegenLinearizerTest, JITCompilationTest)
{
    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar Scalar;
    typedef typename Scalar::value_type AD_ValueType;
    typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, double> A_type;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystem> oscillator(new TestNonlinearSystem(w_n));
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    SystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);
    ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

    // do just in time compilation of the Jacobians
    std::cout << "compiling..." << std::endl;
    adLinearizer.compileJIT("ADCGCodegenLib");
    std::cout << "... done!" << std::endl;

    std::shared_ptr<ADCodegenLinearizer<state_dim, control_dim>> adLinearizerClone(adLinearizer.clone());
    std::cout << "compiling the clone..." << std::endl;
    adLinearizerClone->compileJIT("ADCGCodegenLibCone");
    std::cout << "... done!" << std::endl;

    // create state, control and time variables
    StateVector<TestNonlinearSystem::STATE_DIM> x;
    ControlVector<TestNonlinearSystem::CONTROL_DIM> u;
    double t = 0;

    for (size_t i = 0; i < 1000; i++)
    {
        // set a random state
        x.setRandom();
        u.setRandom();

        // use the numerical differentiation linearizer
        A_type A_system = systemLinearizer.getDerivativeState(x, u, t);
        B_type B_system = systemLinearizer.getDerivativeControl(x, u, t);

        // use the auto diff codegen linearzier
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


/*!
 * Just-in-time compilation test : without compilation of cloned instance
 */
TEST(ADCodegenLinearizerTest, JITCloneTest)
{
    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar Scalar;
    typedef typename Scalar::value_type AD_ValueType;
    typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, double> A_type;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystem> oscillator(new TestNonlinearSystem(w_n));
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    SystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);
    ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

    // do just in time compilation of the Jacobians
    std::cout << "compiling..." << std::endl;
    adLinearizer.compileJIT("ADCGCodegenLib");
    std::cout << "... done!" << std::endl;

    // compilation should not even be required
    std::cout << "cloning without compilation..." << std::endl;
    std::shared_ptr<ADCodegenLinearizer<state_dim, control_dim>> adLinearizerClone(adLinearizer.clone());

    // make sure the underlying dynamic libraries are not identical (dynamic library cloned correctly)
    if (adLinearizerClone->getLinearizer().getDynamicLib() == adLinearizer.getLinearizer().getDynamicLib())
    {
        std::cout << "FATAL ERROR: dynamic library not cloned correctly in JIT." << std::endl;
        ASSERT_TRUE(false);
    }

    // create state, control and time variables
    StateVector<TestNonlinearSystem::STATE_DIM> x;
    ControlVector<TestNonlinearSystem::CONTROL_DIM> u;
    double t = 0;

    for (size_t i = 0; i < 1000; i++)
    {
        // set a random state
        x.setRandom();
        u.setRandom();

        // use the numerical differentiation linearizer
        A_type A_system = systemLinearizer.getDerivativeState(x, u, t);
        B_type B_system = systemLinearizer.getDerivativeControl(x, u, t);

        // use the auto diff codegen linearzier
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


/*!
 * Code generation test for writing code to file
 */
TEST(ADCodegenLinearizerTest, CodegenTest)
{
    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar Scalar;
    typedef typename Scalar::value_type AD_ValueType;
    typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

    // create an auto-differentiable codegen system
    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

    // create a linearizer that uses codegeneration
    ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

    try
    {
        std::cout << "generating code..." << std::endl;
        // generate code for the Jacobians
        adLinearizer.generateCode("TestNonlinearSystemLinearized");
        std::cout << "... done!" << std::endl;
    } catch (const std::runtime_error& e)
    {
        std::cout << "code generation failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}

TEST(ADCodegenLinearizerTestMP, JITCompilationTestMP)
{
    const size_t state_dim = TestNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar Scalar;
    typedef typename Scalar::value_type AD_ValueType;
    typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

    // handy typedefs for the Jacobian
    typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
    typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

    // // create two nonlinear systems, one regular one and one auto-differentiable
    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystem> oscillator(new TestNonlinearSystem(w_n));
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

    // // create two nonlinear systems, one regular one and one auto-diff codegen
    SystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);

    ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);
    adLinearizer.compileJIT("ADMPTestLib");

    size_t nThreads = 4;
    std::vector<std::shared_ptr<ADCodegenLinearizer<state_dim, control_dim>>> adLinearizers;
    std::vector<std::shared_ptr<SystemLinearizer<state_dim, control_dim>>> systemLinearizers;

    for (size_t i = 0; i < nThreads; ++i)
    {
        adLinearizers.push_back(std::shared_ptr<ADCodegenLinearizer<state_dim, control_dim>>(adLinearizer.clone()));
        adLinearizers.back()->compileJIT();
        systemLinearizers.push_back(
            std::shared_ptr<SystemLinearizer<state_dim, control_dim>>(systemLinearizer.clone()));
    }


    size_t runs = 100000;

    for (size_t n = 0; n < runs; ++n)
    {
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nThreads; ++i)
        {
            threads.push_back(std::thread([i, &adLinearizers, &systemLinearizers]() {
                StateVector<TestNonlinearSystem::STATE_DIM> x;
                ControlVector<TestNonlinearSystem::CONTROL_DIM> u;
                double t = 0;

                x.setRandom();
                u.setRandom();

                // use the numerical differentiation linearizer
                A_type A_system = systemLinearizers[i]->getDerivativeState(x, u, t);
                B_type B_system = systemLinearizers[i]->getDerivativeControl(x, u, t);

                // use the auto differentiation linearzier
                A_type A_ad = adLinearizers[i]->getDerivativeState(x, u, t);
                B_type B_ad = adLinearizers[i]->getDerivativeControl(x, u, t);

                // verify the result
                ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
                ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);
            }));
        }


        for (auto& thr : threads)
            thr.join();
    }
}
