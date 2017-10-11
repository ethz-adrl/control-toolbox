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

#include <ct/core/core.h>

#include <chrono>

#include <ct/optcon/optcon-prespec.h>

#ifdef MATLAB
    #include <matlabCppInterface/Engine.hpp>
#endif

// Bring in gtest
#include <gtest/gtest.h>


namespace ct{
namespace optcon{
namespace example{

TEST(LQRTest, DARETest)
{

	const size_t stateDim = 2;
	const size_t controlDim = 1;

	Eigen::Matrix<double, stateDim, stateDim> A;
	Eigen::Matrix<double, stateDim, controlDim> B;
	Eigen::Matrix<double, stateDim, stateDim> Q;
	Eigen::Matrix<double, controlDim, controlDim> R;
	Eigen::Matrix<double, controlDim, stateDim> K;

	A << 1, 1, 1, 0;
	B << 0 , 1;
	Q << 1, 0, 0, 1;
	R << 1;

	ct::optcon::DARE<stateDim, controlDim> dare;
	Eigen::Matrix<double, stateDim, stateDim> P = dare.computeSteadyStateRiccatiMatrix(Q, R, A, B, K, true);
}


TEST(LQRTest, quadTest)
{
//	std::cout << "QUADROTOR TEST"<<std::endl;
//	std::cout << "==================================="<<std::endl;
//	std::cout << "==================================="<<std::endl << std::endl << std::endl;

	const size_t stateDim = 12;
	const size_t controlDim = 4;

	Eigen::Matrix<double, stateDim, stateDim> A;
	Eigen::Matrix<double, stateDim, controlDim> B;
	Eigen::Matrix<double, stateDim, stateDim> Q;
	Eigen::Matrix<double, controlDim, controlDim> R;
	Eigen::Matrix<double, controlDim, stateDim> K;
	Eigen::Matrix<double, controlDim, stateDim> Kiterative;

	ct::optcon::LQR<stateDim, controlDim> lqr;

	Q <<
		  10,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
		   0,   10,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
		   0,    0, 2000,    0,    0,    0,    0,    0,    0,    0,    0,    0,
		   0,    0,    0,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,
		   0,    0,    0,    0,  0.1,    0,    0,    0,    0,    0,    0,    0,
		   0,    0,    0,    0,    0,  0.5,    0,    0,    0,    0,    0,    0,
		   0,    0,    0,    0,    0,    0,  0.1,    0,    0,    0,    0,    0,
		   0,    0,    0,    0,    0,    0,    0,  0.1,    0,    0,    0,    0,
		   0,    0,    0,    0,    0,    0,    0,    0,    1,    0,    0,    0,
		   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.2,    0,    0,
		   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  0.2,    0,
		   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 0.02;

	R <<
		 100,    0,    0,    0,
		   0, 1000,    0,    0,
		   0,    0, 1000,    0,
		   0,    0,    0,  100;


	A <<
		0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,
	    0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,
	    0,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,
	    0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,
	    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0,
	    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,
	    0,     0,     0,     0,  9.81,     0,     0,     0,     0,     0,     0,     0,
	    0,     0,     0, -9.81,     0,     0,     0,     0,     0,     0,     0,     0,
	    0,     0,     0,    -0,    -0,     0,     0,     0,     0,     0,     0,     0,
	    0,     0,     0,     0,     0,    -0,     0,     0,     0,     0,    -0,    -0,
	    0,     0,     0,     0,    -0,    -0,     0,     0,     0,     0,     0,     0,
	    0,     0,     0,     0,     0,     0,     0,     0,     0,    -0,     0,     0;


	B <<
		  0,       0,       0,       0,
	      0,       0,       0,       0,
	      0,       0,       0,       0,
	      0,       0,       0,       0,
	      0,       0,       0,       0,
	      0,       0,       0,       0,
	      0,       0,       0,       0,
	     -0,       0,       0,       0,
	1.39665,       0,       0,       0,
	      0, 142.857,      -0,       0,
	      0,       0, 142.857,       0,
	      0,      -0,       0, 83.3333;

	Kiterative.setZero();
	K.setZero();


	bool foundSolutionIterative = lqr.compute(Q, R, A, B, Kiterative, false, true);
	ASSERT_TRUE(foundSolutionIterative);

	bool foundSolutionDirect = lqr.compute(Q, R, A, B, K, false);
	ASSERT_TRUE(foundSolutionDirect);

	ASSERT_LT((K - Kiterative).array().abs().maxCoeff(), 1e-4);

	int nTests = 1000;
	auto start = std::chrono::system_clock::now();
	for (int i=0; i<nTests; i++)
	{
		lqr.compute(Q, R, A, B, K, false);
	}
	auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "solved "<<nTests<<" lqr problems with state dimension "<<stateDim<<" in "<<elapsed.count()<<" ms (average: "<<elapsed.count()/static_cast<double>(nTests)<<" ms / lqr)"<<std::endl;

	start = std::chrono::system_clock::now();
	for (int i=0; i<nTests; i++)
	{
		lqr.compute(Q, R, A, B, Kiterative, false, true);
	}
	end = std::chrono::system_clock::now();
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "solved "<<nTests<<" lqr problems iteratively with state dimension "<<stateDim<<" in "<<elapsed.count()<<" ms (average: "<<elapsed.count()/static_cast<double>(nTests)<<" ms / lqr)"<<std::endl;

}

#ifdef MATLAB
TEST(LQRTest, matlabTest)
{
	matlab::Engine engine(true);
	ASSERT_TRUE(engine.good());

	const size_t stateDim = 5;
	std::string stateDimString = std::to_string(stateDim);

	Eigen::MatrixXd Ad;
	Eigen::MatrixXd Bd;
	Eigen::MatrixXd Qd;
	Eigen::MatrixXd Rd;
	Eigen::MatrixXd K_Matlab;

	Eigen::Matrix<double, stateDim, stateDim> A;
	Eigen::Matrix<double, stateDim, stateDim> B;
	Eigen::Matrix<double, stateDim, stateDim> Q;
	Eigen::Matrix<double, stateDim, stateDim> R;
	Eigen::Matrix<double, stateDim, stateDim> K_Cpp;
	Eigen::Matrix<double, stateDim, stateDim> K_Cpp_iteratively;

	ct::optcon::LQR<stateDim, stateDim> lqr;

//	std::cout << "ARTIFICIAL TEST "<<std::endl;
//	std::cout << "==================================="<<std::endl;
//	std::cout << "==================================="<<std::endl << std::endl << std::endl;

	for (int i=0; i<10; i++)
	{
		std::cout << "Test "<< std::to_string(i)<<std::endl;
		std::cout << "==================================="<<std::endl;

		std::cout << "1. Generating problem in Matlab" << std::endl;
		engine.executeCommand("A = magic(" + stateDimString + ");");
		engine.executeCommand("B = magic(" + stateDimString + ");");
		engine.executeCommand("Q = diag(100*rand(" + stateDimString + ",1));");
		engine.executeCommand("R = diag(100*rand(" + stateDimString + ",1));");
		engine.executeCommand("N = zeros(" + stateDimString + ");");

		std::cout << "2. Computing LQR in Matlab" << std::endl;
		std::cout << engine.executeCommand("[K,S,E] = lqr(A,B,Q,R,N);");

		std::cout << "3. Obtaining problem from Matlab" << std::endl;
		engine.get("A", Ad); A = Ad;
		engine.get("B", Bd); B = Bd;
		engine.get("Q", Qd); Q = Qd;
		engine.get("R", Rd); R = Rd;

		std::cout << "4. Obtaining LQR solution from Matlab" << std::endl;
		engine.get("K", K_Matlab);

		std::cout << "5. Computing LQR solution in C++" << std::endl;

        bool foundSolutionDirect = lqr.compute(Q, R, A, B, K_Cpp, false);
        ASSERT_TRUE(foundSolutionDirect);

		bool foundSolutionIterative = lqr.compute(Q, R, A, B, K_Cpp_iteratively, false, true);
        ASSERT_TRUE(foundSolutionIterative);


		std::cout << "7. Comparing both solutions" << std::endl;
		ASSERT_LT((K_Matlab - K_Cpp).array().abs().maxCoeff(), 1e-4);
		ASSERT_LT((K_Matlab - K_Cpp_iteratively).array().abs().maxCoeff(), 1e-4);

		std::cout << std::endl << std::endl << std::endl;
	}
}
#endif //MATLAB

} // namespace example
} // namespace optcon
} // namespace ct


/*!
 * This runs the LQR unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example LqrTest.cpp
 */
int main(int argc, char **argv)
{
  using namespace ct::optcon::example;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
