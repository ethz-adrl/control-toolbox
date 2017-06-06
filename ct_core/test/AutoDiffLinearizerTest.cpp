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

#include <iosfwd>
#include <vector>
#include <cppad/cg.hpp>

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/example/eigen_mat_inv.hpp>

#include <cmath>
#include <memory>


#include <ct/core/core.h>
#include "system/TestNonlinearSystem.h"

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
	const size_t state_dim = TestNonlinearSystem::STATE_DIM;
	const size_t control_dim = TestNonlinearSystem::CONTROL_DIM;

	typedef CppAD::AD<double> AD_Scalar;
	typedef tpl::TestNonlinearSystem<AD_Scalar> TestNonlinearSystemAD;

	typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
	typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

	double w_n = 100;
	shared_ptr<TestNonlinearSystem > nonlinearSystem(new TestNonlinearSystem(w_n));
	shared_ptr<TestNonlinearSystemAD> nonlinearSystemAD(new tpl::TestNonlinearSystem<AD_Scalar>(AD_Scalar(w_n)));

	SystemLinearizer<state_dim, control_dim> systemLinearizer(nonlinearSystem);
	AutoDiffLinearizer<state_dim, control_dim> adLinearizer(nonlinearSystemAD);

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

//		std::cout << "A_system: "<<std::endl<<A_system<<std::endl<<std::endl;
//		std::cout << "A_ad: "<<std::endl<<A_ad<<std::endl<<std::endl;
//
//		std::cout << "B_system: "<<std::endl<<B_system<<std::endl<<std::endl;
//		std::cout << "B_ad: "<<std::endl<<B_ad<<std::endl<<std::endl;

		ASSERT_LT((A_system - A_ad).array().abs().maxCoeff(), 1e-5);
		ASSERT_LT((B_system - B_ad).array().abs().maxCoeff(), 1e-5);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
