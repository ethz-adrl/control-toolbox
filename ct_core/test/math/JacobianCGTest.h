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


const size_t inDim = 3;
const size_t outDim = 2;

typedef JacobianCG<inDim, outDim> JacCG;

// gets run by CodegenTests.cpp to ensure all codegen tests do NOT run in parallel

template <typename SCALAR>
Eigen::Matrix<SCALAR, outDim, 1> testFunction(const Eigen::Matrix<SCALAR, inDim, 1>& x)
{
	Eigen::Matrix<SCALAR, outDim, 1> y;

	y(0) = 3*x(0) + 2*x(0)*x(0) - x(1)*x(2);
	y(1) = x(2) + x(1) + 3;

	return y;
}

template <typename SCALAR>
Eigen::Matrix<SCALAR, outDim, inDim> jacobianCheck(const Eigen::Matrix<SCALAR, inDim, 1>& x)
{
	Eigen::Matrix<SCALAR, outDim, inDim> jac;

	jac << 3+4*x(0),    -x(2),     -x(1),
		   0,              1,        1;

	return jac;
}

TEST(JacobianCGTest, JITCompilationTest)
{
	try {
		typename JacCG::Function f = testFunction<CppAD::AD<CppAD::cg::CG<double> > >;

		JacCG jacCG(f);

		jacCG.compileJIT();

		Eigen::Matrix<double, inDim, 1> x;

		for (size_t i=0; i<1000; i++)
		{
			x.setRandom();

	//		std::cout << "jacCG(x): " << std::endl << jacCG(x) << std::endl;
	//		std::cout << "jacobianCheck(x): " << std::endl << jacobianCheck(x) << std::endl;

			ASSERT_LT((jacCG(x) - jacobianCheck(x)).array().abs().maxCoeff(), 1e-10);
		}
	} catch (std::exception& e)
	{
		std::cout << "Exception thrown: "<<e.what()<<std::endl;
		ASSERT_TRUE(false);
	}
}



TEST(JacobianCGTest, CodegenTest)
{
	typename JacCG::Function f = testFunction<CppAD::AD<CppAD::cg::CG<double> > >;

	JacCG jacCG(f);

	jacCG.generateCode("TestJacobian");

	jacCG.generateForwardZeroCode("TestForwardZero");
}

