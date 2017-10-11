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

#include <ct/optcon/optcon.h>

#include <ct/costfunction/CostFunctionAD.hpp>
#include <ct/costfunction/CostFunctionAnalytical.hpp>
#include <ct/costfunction/term/TermBase.hpp>
#include <ct/costfunction/term/TermLinear.hpp>
#include <ct/costfunction/term/TermQuadratic.hpp>
#include <ct/costfunction/term/TermOther.hpp>


/*!
 * This example illustrates how to use the AD cost function type and compares it to an analytical counterpart.
 * \example ADTest.cpp
 *
 * \todo make this a unit test.
 */
int main() {
	using namespace ct;

	// autodiff costfunction
	std::shared_ptr<CostFunctionAD<3, 3>> ADcf (new CostFunctionAD<3, 3>());

	// analytical costfunction
	std::shared_ptr<CostFunctionAnalytical<3, 3>> ANAcf (new CostFunctionAnalytical<3, 3>());

	Eigen::Matrix<double, 3, 3> Q;
	Eigen::Matrix<double, 3, 3> R;
	Q.setIdentity();
	R.setIdentity();
	std::shared_ptr< TermQuadratic<3, 3> > term1 (new TermQuadratic<3, 3>(Q, R));
	ADcf->addIntermediateTerm(term1);
	ANAcf->addIntermediateTerm(term1);


	Eigen::Matrix<double, 3, 1> a;
	Eigen::Matrix<double, 3, 1> b;
	a << 1.,2.,3.;
	b << 4.,5.,6.;
	std::shared_ptr< TermLinear<3, 3> > term2 (new TermLinear<3, 3>(a, b));
	ADcf->addIntermediateTerm(term2);
	ANAcf->addIntermediateTerm(term2);

	//	ADcf->loadFromConfigFile("/home/gimarkus/catkin_ct/src/ct_costfunction/config/example.info");
	//	ANAcf->loadFromConfigFile("/home/gimarkus/catkin_ct/src/ct_costfunction/config/example.info");

	Eigen::Vector3d x;
	Eigen::Vector3d u;

	x << -1.0, -2.0, 3.0;
	u << 1.0, 2.3, -3.1;

	double t = 0.0;

	ADcf->setCurrentStateAndControl(x, u, t);
	ANAcf->setCurrentStateAndControl(x, u, t);

	std::cout
	<< "diff evaluateIntermediate() = " << ADcf->evaluateIntermediateCost() -  ANAcf->evaluateIntermediateCost()  << std::endl
	<< "diff intermediateStateDerivative() = " << std::endl << ADcf->intermediateStateDerivative() - ANAcf->intermediateStateDerivative()  << std::endl
	<< "diff intermediateStateSecondDerivative() = " << std::endl << ADcf->intermediateStateSecondDerivative() - ANAcf->intermediateStateSecondDerivative() << std::endl
	<< "diff intermediateControlDerivative() = " << std::endl << ADcf->intermediateControlDerivative() - ANAcf->intermediateControlDerivative()  << std::endl
	<< "diff intermediateControlSecondDerivative() = " << std::endl << ADcf->intermediateControlSecondDerivative()- ANAcf->intermediateControlSecondDerivative()  << std::endl
	<< "diff intermediateStateControlDerivative() = " << std::endl << ADcf->intermediateStateControlDerivative() - ANAcf->intermediateStateControlDerivative() << std::endl;



	std::shared_ptr<CostFunctionAD<3, 3>> ADclone (std::static_pointer_cast<CostFunctionAD<3, 3>>(ADcf->clone()));

	std::cout
	<< "diff evaluateIntermediate() = " << ADclone->evaluateIntermediateCost() -  ANAcf->evaluateIntermediateCost()  << std::endl
	<< "diff intermediateStateDerivative() = " << std::endl << ADclone->intermediateStateDerivative() - ANAcf->intermediateStateDerivative()  << std::endl
	<< "diff intermediateStateSecondDerivative() = " << std::endl << ADclone->intermediateStateSecondDerivative() - ANAcf->intermediateStateSecondDerivative() << std::endl
	<< "diff intermediateControlDerivative() = " << std::endl << ADclone->intermediateControlDerivative() - ANAcf->intermediateControlDerivative()  << std::endl
	<< "diff intermediateControlSecondDerivative() = " << std::endl << ADclone->intermediateControlSecondDerivative()- ANAcf->intermediateControlSecondDerivative()  << std::endl
	<< "diff intermediateStateControlDerivative() = " << std::endl << ADclone->intermediateStateControlDerivative() - ANAcf->intermediateStateControlDerivative() << std::endl;


	return 0;
}


