/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TestDiscreteNonlinearSystem.h"

/*!
 * Just-in-time compilation test
 */
TEST(DiscreteSystemLinearizerADCG, JITCompilationTest)
{
    // define the dimensions of the system
    const size_t state_dim = TestDiscreteNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestDiscreteNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef DiscreteSystemLinearizerADCG<state_dim, control_dim>::ADCGScalar ADCGScalar;
    typedef typename ADCGScalar::value_type AD_ValueType;
    typedef tpl::TestDiscreteNonlinearSystem<ADCGScalar> TestDiscreteNonlinearSystemAD;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, double> A_type;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double rate = 100.0;
    shared_ptr<TestDiscreteNonlinearSystem> oscillator(new TestDiscreteNonlinearSystem(rate));
    shared_ptr<TestDiscreteNonlinearSystemAD> oscillatorAD(
        new tpl::TestDiscreteNonlinearSystem<ADCGScalar>(AD_ValueType(rate)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    DiscreteSystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);
    DiscreteSystemLinearizerADCG<state_dim, control_dim> adLinearizer(oscillatorAD);

    // do just in time compilation of the Jacobians
    std::cout << "compiling..." << std::endl;
    adLinearizer.compileJIT("ADCGCodegenLib");
    std::cout << "... done!" << std::endl;

    std::shared_ptr<DiscreteSystemLinearizerADCG<state_dim, control_dim>> adLinearizerClone(adLinearizer.clone());

    // make sure the underlying dynamic libraries are not identical (dynamic library cloned correctly)
    if (adLinearizerClone->getLinearizer().getDynamicLib() == adLinearizer.getLinearizer().getDynamicLib())
    {
        std::cout << "FATAL ERROR: dynamic library not cloned correctly in JIT." << std::endl;
        ASSERT_TRUE(false);
    }

    // create state, control and time variables
    StateVector<state_dim> x;
    ControlVector<control_dim> u;
    int n = 0;

    for (size_t i = 0; i < 1000; i++)
    {
        // set a random state
        x.setRandom();
        u.setRandom();

        // use the numerical differentiation linearizer
        A_type A_system;
        B_type B_system;
        systemLinearizer.getAandB(x, u, x, n, 1, A_system, B_system);

        // use the auto diff codegen linearzier
        A_type A_ad;
        B_type B_ad;
        adLinearizer.getAandB(x, u, x, n, 1, A_ad, B_ad);

        A_type A_adCloned;
        B_type B_adCloned;
        adLinearizerClone->getAandB(x, u, x, n, 1, A_adCloned, B_adCloned);

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
TEST(DiscreteSystemLinearizerADCG, CodegenTest)
{
    // define the dimensions of the system
    const size_t state_dim = TestDiscreteNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestDiscreteNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef DiscreteSystemLinearizerADCG<state_dim, control_dim>::ADCGScalar ADCGScalar;
    typedef typename ADCGScalar::value_type AD_ValueType;
    typedef tpl::TestDiscreteNonlinearSystem<ADCGScalar> TestDiscreteNonlinearSystemAD;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double rate = 100.0;
    shared_ptr<TestDiscreteNonlinearSystemAD> oscillatorAD(
        new tpl::TestDiscreteNonlinearSystem<ADCGScalar>(AD_ValueType(rate)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    DiscreteSystemLinearizerADCG<state_dim, control_dim> adLinearizer(oscillatorAD);

    try
    {
        std::cout << "generating code..." << std::endl;
        // generate code for the Jacobians
        adLinearizer.generateCode("TestDiscreteNonlinearSystemLinearized");
        std::cout << "... done!" << std::endl;
    } catch (const std::runtime_error& e)
    {
        std::cout << "code generation failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}

TEST(DiscreteSystemLinearizerADCGMP, JITCompilationTestMP)
{
    // define the dimensions of the system
    const size_t state_dim = TestDiscreteNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestDiscreteNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef DiscreteSystemLinearizerADCG<state_dim, control_dim>::ADCGScalar ADCGScalar;
    typedef typename ADCGScalar::value_type AD_ValueType;
    typedef tpl::TestDiscreteNonlinearSystem<ADCGScalar> TestDiscreteNonlinearSystemAD;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, double> A_type;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double rate = 100.0;
    shared_ptr<TestDiscreteNonlinearSystem> oscillator(new TestDiscreteNonlinearSystem(rate));
    shared_ptr<TestDiscreteNonlinearSystemAD> oscillatorAD(
        new tpl::TestDiscreteNonlinearSystem<ADCGScalar>(AD_ValueType(rate)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    DiscreteSystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);
    DiscreteSystemLinearizerADCG<state_dim, control_dim> adLinearizer(oscillatorAD);
    adLinearizer.compileJIT("ADMPTestLib");

    size_t nThreads = 4;
    std::vector<std::shared_ptr<DiscreteSystemLinearizerADCG<state_dim, control_dim>>> adLinearizers;
    std::vector<std::shared_ptr<DiscreteSystemLinearizer<state_dim, control_dim>>> systemLinearizers;

    for (size_t i = 0; i < nThreads; ++i)
    {
        adLinearizers.push_back(
            std::shared_ptr<DiscreteSystemLinearizerADCG<state_dim, control_dim>>(adLinearizer.clone()));
        adLinearizers.back()->compileJIT();
        systemLinearizers.push_back(
            std::shared_ptr<DiscreteSystemLinearizer<state_dim, control_dim>>(systemLinearizer.clone()));
    }


    size_t runs = 100000;

    for (size_t n = 0; n < runs; ++n)
    {
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nThreads; ++i)
        {
            threads.push_back(std::thread([i, &adLinearizers, &systemLinearizers]() {
                StateVector<TestDiscreteNonlinearSystem::STATE_DIM> x;
                ControlVector<TestDiscreteNonlinearSystem::CONTROL_DIM> u;
                int n = 0;

                x.setRandom();
                u.setRandom();

                // use the numerical differentiation linearizer
                A_type A_system;
                B_type B_system;
                systemLinearizers[i]->getAandB(x, u, x, n, 1, A_system, B_system);

                // use the auto diff codegen linearzier
                A_type A_ad;
                B_type B_ad;
                adLinearizers[i]->getAandB(x, u, x, n, 1, A_ad, B_ad);

                // verify the result
                ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
                ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);
            }));
        }


        for (auto& thr : threads)
            thr.join();
    }
}

TEST(DiscreteSystemLinearizerADCG, FloatTest)
{
    // define the dimensions of the system
    const size_t state_dim = TestDiscreteNonlinearSystem::STATE_DIM;
    const size_t control_dim = TestDiscreteNonlinearSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef DiscreteSystemLinearizerADCG<state_dim, control_dim, float>::ADCGScalar ADCGScalarFloat;
    typedef DiscreteSystemLinearizerADCG<state_dim, control_dim, double>::ADCGScalar ADCGScalarDouble;

    typedef typename ADCGScalarFloat::value_type AD_ValueTypeFloat;
    typedef typename ADCGScalarDouble::value_type AD_ValueTypeDouble;

    typedef tpl::TestDiscreteNonlinearSystem<ADCGScalarFloat> TestDiscreteNonlinearSystemADFloat;
    typedef tpl::TestDiscreteNonlinearSystem<ADCGScalarDouble> TestDiscreteNonlinearSystemADDouble;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, float> A_typeFloat;
    typedef ct::core::StateMatrix<state_dim, double> A_typeDouble;

    typedef ct::core::StateControlMatrix<state_dim, control_dim, float> B_typeFloat;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_typeDouble;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const float rateFloat = 100.0;
    const double rateDouble = static_cast<double>(rateFloat);

    shared_ptr<TestDiscreteNonlinearSystemADFloat> oscillatorADFloat(
        new tpl::TestDiscreteNonlinearSystem<ADCGScalarFloat>(AD_ValueTypeFloat(rateFloat)));
    shared_ptr<TestDiscreteNonlinearSystemADDouble> oscillatorADDouble(
        new tpl::TestDiscreteNonlinearSystem<ADCGScalarDouble>(AD_ValueTypeDouble(rateDouble)));

    DiscreteSystemLinearizerADCG<state_dim, control_dim, float> adLinearizerFloat(oscillatorADFloat);
    DiscreteSystemLinearizerADCG<state_dim, control_dim, double> adLinearizerDouble(oscillatorADDouble);

    // do just in time compilation of the Jacobians
    std::cout << "compiling..." << std::endl;
    adLinearizerFloat.compileJIT("ADCGCodegenLibFloat");
    adLinearizerDouble.compileJIT("ADCGCodegenLibDouble");
    std::cout << "... done compiling!" << std::endl;

    // create state, control and time variables
    StateVector<state_dim, float> xFloat;
    StateVector<state_dim, double> xDouble;
    ControlVector<control_dim, float> uFloat;
    ControlVector<control_dim, double> uDouble;
    int n = 0;

    for (size_t i = 0; i < 1000; i++)
    {
        // set a random state
        xFloat.setRandom();
        xDouble = xFloat.cast<double>();
        uFloat.setRandom();
        uDouble = uFloat.cast<double>();

        A_typeFloat A_Float;
        B_typeFloat B_Float;
        adLinearizerFloat.getAandB(xFloat, uFloat, xFloat, n, 1, A_Float, B_Float);

        A_typeDouble A_Double;
        B_typeDouble B_Double;
        adLinearizerDouble.getAandB(xDouble, uDouble, xDouble, n, 1, A_Double, B_Double);

        // verify the result //FIXME: this tolerance is too high
        ASSERT_LT((A_Double - A_Float.cast<double>()).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_Double - B_Float.cast<double>()).array().abs().maxCoeff(), 1e-5);
    }

    try
    {
        std::cout << "generating code..." << std::endl;
        // generate code for the Jacobians
        adLinearizerFloat.generateCode("TestDiscreteNonlinearSystemLinearizedFloat");
        adLinearizerDouble.generateCode("TestDiscreteNonlinearSystemLinearizedDouble");
        std::cout << "... done!" << std::endl;
    } catch (const std::runtime_error& e)
    {
        std::cout << "code generation failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}
