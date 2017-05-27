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

#include "../../models/testhyq/RobCoGenTestHyQ.h"
#include <ct/rbd/robot/costfunction/TermTaskspace.hpp>
#include <ct/optcon/costfunction/CostFunctionAD.hpp>
#include <gtest/gtest.h>


using namespace ct;
using namespace rbd;


TEST(TaskspaceCostFunctionTests, TaskspaceCostFunctionTest)
{
	typedef CppAD::AD<double> size_type;
	typedef TestHyQ::tpl::Kinematics<size_type> KinTpl_t;
	KinTpl_t kynTpl;
	size_t eeId = 1;
	const size_t hyqStateDim = 36;

	Eigen::Matrix<size_type, 3, 3> Q;
	Q.setIdentity();

	std::shared_ptr<optcon::CostFunctionAD<hyqStateDim,12>> Adcf(new optcon::CostFunctionAD<hyqStateDim, 12>());
	std::shared_ptr<TermTaskspace<KinTpl_t, true, hyqStateDim, 12>> term1(new TermTaskspace<KinTpl_t, true, hyqStateDim, 12>(eeId, Q));
	Adcf->addIntermediateTerm(term1);

	Eigen::Matrix<double, hyqStateDim, 1> x;
	Eigen::Matrix<double, 12, 1> u;

	x.setRandom();
	u.setRandom();

	std::cout << "state: " << x.transpose() << std::endl;
	std::cout << "input: " << u.transpose() << std::endl;

	double t = 0.0;

	Adcf->setCurrentStateAndControl(x, u, t);	

	ASSERT_TRUE(Adcf->stateDerivativeIntermediateTest());
	ASSERT_TRUE(Adcf->controlDerivativeIntermediateTest());	

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



