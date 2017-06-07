/*!
 * \example ADCodegenLinearizerTest.h
 *
 * Unit test for using Auto-Diff Codegeneration to compute the Jacobians/Linearization of a nonlinear System
 *
 * \note gets run by CodegenTests.cpp to ensure all codegen tests do NOT run in parallel
 */

#include "TestNonlinearSystem.h"

/*!
 * Just-in-time compilation test
 */
TEST(ADCodegenLinearizerTest, JITCompilationTest)
{
	// define the dimensions of the system
	const size_t state_dim = TestNonlinearSystem::STATE_DIM;
	const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

	// typedefs for the auto-differentiable codegen system
	typedef ADCodegenLinearizer<state_dim, control_dim>::SCALAR Scalar;
	typedef typename Scalar::value_type AD_ValueType;
	typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

	// handy typedefs for the Jacobian
	typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
	typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

	// create two nonlinear systems, one regular one and one auto-differentiable
	const double w_n = 100.0;
	shared_ptr<TestNonlinearSystem > oscillator(new TestNonlinearSystem(w_n));
	shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

	// create two nonlinear systems, one regular one and one auto-diff codegen
	SystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);
	ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

	// do just in time compilation of the Jacobians
	std::cout<< "compiling..." << std::endl;
	adLinearizer.compileJIT();
	std::cout<< "... done!" << std::endl;

	// create state, control and time variables
	StateVector<TestNonlinearSystem::STATE_DIM> x;
	ControlVector<TestNonlinearSystem::CONTROL_DIM> u;
	double t = 0;

	for (size_t i=0; i<1000; i++)
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

		// verify the result
		ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
		ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);
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
	typedef ADCodegenLinearizer<state_dim, control_dim>::SCALAR Scalar;
	typedef typename Scalar::value_type AD_ValueType;
	typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

	// create an auto-differentiable codegen system
	const double w_n = 100.0;
	shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

	// create a linearizer that uses codegeneration
	ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

	try {
		std::cout<< "generating code..." << std::endl;
		// generate code for the Jacobians
		adLinearizer.generateCode("TestNonlinearSystemLinearized");
		std::cout<< "... done!" << std::endl;
	} catch (const std::runtime_error& e)
	{
		std::cout << "code generation failed: "<<e.what()<<std::endl;
		ASSERT_TRUE(false);
	}
}

