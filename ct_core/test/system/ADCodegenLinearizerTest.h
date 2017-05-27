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


// gets run by CodegenTests.cpp to ensure all codegen tests do NOT run in parallel
#include "TestNonlinearSystem.h"

TEST(ADCodegenLinearizerTest, JITCompilationTest)
{
	const size_t state_dim = TestNonlinearSystem::STATE_DIM;
	const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

	typedef ADCodegenLinearizer<state_dim, control_dim>::SCALAR Scalar;
	typedef typename Scalar::value_type AD_ValueType;
	typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

	typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
	typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

	const double w_n = 100.0;
	shared_ptr<TestNonlinearSystem > oscillator(new TestNonlinearSystem(w_n));
	shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

	SystemLinearizer<state_dim, control_dim> systemLinearizer(oscillator);
	ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

	std::cout<< "compiling..." << std::endl;
	adLinearizer.compileJIT();
	std::cout<< "... done!" << std::endl;

	StateVector<TestNonlinearSystem::STATE_DIM> x;
	ControlVector<TestNonlinearSystem::CONTROL_DIM> u;
	double t = 0;

	for (size_t i=0; i<1000; i++)
	{
		x.setRandom();
		u.setRandom();

		A_type A_system = systemLinearizer.getDerivativeState(x, u, t);
		B_type B_system = systemLinearizer.getDerivativeControl(x, u, t);

		A_type A_ad = adLinearizer.getDerivativeState(x, u, t);
		B_type B_ad = adLinearizer.getDerivativeControl(x, u, t);

		ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
		ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);
	}

}



TEST(ADCodegenLinearizerTest, CodegenTest)
{
	const size_t state_dim = TestNonlinearSystem::STATE_DIM;
	const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

	typedef ADCodegenLinearizer<state_dim, control_dim>::SCALAR Scalar;
	typedef typename Scalar::value_type AD_ValueType;
	typedef tpl::TestNonlinearSystem<Scalar> TestNonlinearSystemAD;

	const double w_n = 100.0;
	shared_ptr<TestNonlinearSystem > oscillator(new TestNonlinearSystem(w_n));
	shared_ptr<TestNonlinearSystemAD> oscillatorAD(new tpl::TestNonlinearSystem<Scalar>(AD_ValueType(w_n)));

	ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorAD);

	try {
	std::cout<< "generating code..." << std::endl;
		adLinearizer.generateCode("TestNonlinearSystemLinearized");
	std::cout<< "... done!" << std::endl;
	} catch (const std::runtime_error& e)
	{
		std::cout << "code generation failed: "<<e.what()<<std::endl;
		ASSERT_TRUE(false);
	}
}

//int main(int argc, char **argv){
//  testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
//}
