/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>
#include "TestNonlinearSystem.h"

/*!
 * Just-in-time compilation test : compile cloned instance
 */
template <ct::core::TIME_TYPE TIME_T>
void runJitCompilationTests()
{
    // typedefs for the auto-differentiable system
    using ADCGScalar = CppAD::AD<CppAD::cg::CG<double>>;  //!< Autodiff codegen type
    typedef typename ADCGScalar::value_type ADCG_ValueType;
    typedef tpl::TestNonlinearSystem<double, TIME_T> TestNonlinearSystemD;
    typedef tpl::TestNonlinearSystem<ADCGScalar, TIME_T> TestNonlinearSystemAD;

    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystemD::STATE_DIM;
    const size_t control_dim = TestNonlinearSystemD::CONTROL_DIM;
    using State = EuclideanState<state_dim, double>;
    using ADCG_State = EuclideanState<state_dim, ADCGScalar>;

    using ADCodegenLinearizer_t = ADCodegenLinearizer<State, ADCG_State, control_dim, TIME_T>;
    using SystemLinearizer_t = SystemLinearizer<State, control_dim, TIME_T>;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, double> A_type;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystemD> oscillator(new TestNonlinearSystemD(w_n));
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new TestNonlinearSystemAD(ADCG_ValueType(w_n)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    SystemLinearizer_t systemLinearizer(oscillator);
    ADCodegenLinearizer_t adLinearizer(oscillatorAD);

    // do just in time compilation of the Jacobians
    adLinearizer.compileJIT("ADCGCodegenLib");

    // -- Test independent compilation for the cloned linearizer
    std::shared_ptr<ADCodegenLinearizer_t> adLinearizerClone_with_comp(adLinearizer.clone());
    adLinearizerClone_with_comp->compileJIT("ADCGCodegenLibClone");

    // -- Test cloning of existing lib without compilation
    std::shared_ptr<ADCodegenLinearizer_t> adLinearizerClone_no_comp(adLinearizer.clone());

    // -- make sure the underlying dynamic libraries are not identical (dynamic library cloned correctly)
    if (adLinearizerClone_no_comp->getLinearizer().getDynamicLib() == adLinearizer.getLinearizer().getDynamicLib())
    {
        std::cout << "FATAL ERROR: dynamic library not cloned correctly in JIT." << std::endl;
        ASSERT_TRUE(false);
    }

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

        // use the auto diff codegen linearzier
        A_type A_ad = adLinearizer.getDerivativeState(x, u, t);
        B_type B_ad = adLinearizer.getDerivativeControl(x, u, t);

        A_type A_adCloned = adLinearizerClone_with_comp->getDerivativeState(x, u, t);
        B_type B_adCloned = adLinearizerClone_with_comp->getDerivativeControl(x, u, t);

        A_type A_adCloned2 = adLinearizerClone_no_comp->getDerivativeState(x, u, t);
        B_type B_adCloned2 = adLinearizerClone_no_comp->getDerivativeControl(x, u, t);

        // verify the result
        ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);

        ASSERT_LT((A_system - A_adCloned).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_adCloned).array().abs().maxCoeff(), 1e-5);

        ASSERT_LT((A_system - A_adCloned2).array().abs().maxCoeff(), 1e-5);
        ASSERT_LT((B_system - B_adCloned2).array().abs().maxCoeff(), 1e-5);
    }
}
TEST(ADCodegenLinearizerTest, JITCompilationTest_continous_time)
{
    runJitCompilationTests<ct::core::CONTINUOUS_TIME>();
}
TEST(ADCodegenLinearizerTest, JITCompilationTest_discrete_time)
{
    runJitCompilationTests<ct::core::DISCRETE_TIME>();
}


/*!
 * Code generation test for writing code to file
 */
template <ct::core::TIME_TYPE TIME_T>
void runCodeGenerationTests()
{
    // typedefs for the auto-differentiable system
    using ADCGScalar = CppAD::AD<CppAD::cg::CG<double>>;  //!< Autodiff codegen type
    typedef typename ADCGScalar::value_type ADCG_ValueType;
    typedef tpl::TestNonlinearSystem<ADCGScalar, TIME_T> TestNonlinearSystemAD;

    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystemAD::STATE_DIM;
    const size_t control_dim = TestNonlinearSystemAD::CONTROL_DIM;
    using State = EuclideanState<state_dim, double>;
    using ADCG_State = EuclideanState<state_dim, ADCGScalar>;

    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new TestNonlinearSystemAD(ADCG_ValueType(w_n)));

    using ADCodegenLinearizer_t = ADCodegenLinearizer<State, ADCG_State, control_dim, TIME_T>;
    ADCodegenLinearizer_t adLinearizer(oscillatorAD);

    try
    {
        // generate code for the Jacobians
        adLinearizer.generateCode("TestNonlinearSystemLinearized");
    } catch (const std::runtime_error& e)
    {
        std::cout << "code generation failed: " << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}
TEST(ADCodegenLinearizerTest, CodegenTest_continuous_time)
{
    runCodeGenerationTests<ct::core::CONTINUOUS_TIME>();
}
TEST(ADCodegenLinearizerTest, CodegenTest_discrete_time)
{
    runCodeGenerationTests<ct::core::DISCRETE_TIME>();
}


template <ct::core::TIME_TYPE TIME_T>
void runJitCompilationTestsMultithread()
{
    // typedefs for the auto-differentiable system
    using ADCGScalar = CppAD::AD<CppAD::cg::CG<double>>;  //!< Autodiff codegen type
    typedef typename ADCGScalar::value_type ADCG_ValueType;
    typedef tpl::TestNonlinearSystem<double, TIME_T> TestNonlinearSystemD;
    typedef tpl::TestNonlinearSystem<ADCGScalar, TIME_T> TestNonlinearSystemAD;

    // define the dimensions of the system
    const size_t state_dim = TestNonlinearSystemD::STATE_DIM;
    const size_t control_dim = TestNonlinearSystemD::CONTROL_DIM;
    using State = EuclideanState<state_dim, double>;
    using ADCG_State = EuclideanState<state_dim, ADCGScalar>;

    using ADCodegenLinearizer_t = ADCodegenLinearizer<State, ADCG_State, control_dim, TIME_T>;
    using SystemLinearizer_t = SystemLinearizer<State, control_dim, TIME_T>;

    // handy typedefs for the Jacobian
    typedef ct::core::StateMatrix<state_dim, double> A_type;
    typedef ct::core::StateControlMatrix<state_dim, control_dim, double> B_type;

    // create two nonlinear systems, one regular one and one auto-differentiable
    const double w_n = 100.0;
    shared_ptr<TestNonlinearSystemD> oscillator(new TestNonlinearSystemD(w_n));
    shared_ptr<TestNonlinearSystemAD> oscillatorAD(new TestNonlinearSystemAD(ADCG_ValueType(w_n)));

    // create two nonlinear systems, one regular one and one auto-diff codegen
    SystemLinearizer_t systemLinearizer(oscillator);
    ADCodegenLinearizer_t adLinearizer(oscillatorAD);
    adLinearizer.compileJIT("ADMPTestLib");

    size_t nThreads = 4;
    std::vector<std::shared_ptr<ADCodegenLinearizer_t>> adLinearizers;
    std::vector<std::shared_ptr<SystemLinearizer_t>> systemLinearizers;

    for (size_t i = 0; i < nThreads; ++i)
    {
        adLinearizers.push_back(std::shared_ptr<ADCodegenLinearizer_t>(adLinearizer.clone()));
        adLinearizers.back()->compileJIT();
        systemLinearizers.push_back(std::shared_ptr<SystemLinearizer_t>(systemLinearizer.clone()));
    }


    size_t runs = 10000;

    for (size_t n = 0; n < runs; ++n)
    {
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nThreads; ++i)
        {
            threads.push_back(std::thread([i, &adLinearizers, &systemLinearizers]() {
                State x;
                ControlVector<TestNonlinearSystemD::CONTROL_DIM> u;
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
TEST(ADCodegenLinearizerTestMP, JITCompilationTestMP_discrete_time)
{
    runJitCompilationTestsMultithread<ct::core::CONTINUOUS_TIME>();
}
TEST(ADCodegenLinearizerTestMP, JITCompilationTestMP_continuous_time)
{
    runJitCompilationTestsMultithread<ct::core::DISCRETE_TIME>();
}
