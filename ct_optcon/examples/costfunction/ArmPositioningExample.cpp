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


#include <ct/optcon/costfunction/CostFunctionAD.hpp>
#include <ct/optcon/costfunction/term/TermBase.hpp>
#include <ct/optcon/costfunction/term/TermOther.hpp>

#include "../../examples/costfunction/EEDistanceTerm.h"

int main() {
	using namespace ct;

	// autodiff costfunction
	std::shared_ptr<CostFunctionAD<stateDim_planar, controlDim_planar>> ADcostFun (new CostFunctionAD<stateDim_planar, controlDim_planar>());

	Eigen::Matrix3d Q_ee; Q_ee.setIdentity();
	Eigen::Matrix3d Q_ee_f; Q_ee_f.setIdentity(); Q_ee_f *= 100;

	Eigen::Vector3d desiredEEPos; desiredEEPos << 1.0, 1.0, 1.0;

	std::shared_ptr< EEDistanceTerm> term1 (new EEDistanceTerm(desiredEEPos, Q_ee));
	ADcostFun->addIntermediateTerm(term1);

	std::shared_ptr< EEDistanceTerm> term1_final (new EEDistanceTerm(desiredEEPos, Q_ee_f));
	ADcostFun->addFinalTerm(term1_final);


	Eigen::Vector2d x;
	Eigen::Matrix<double, controlDim_planar, 1> u;

	x.setZero();
	u.setZero();

	double t = 0.0;

	ADcostFun->setCurrentStateAndControl(x, u, t);

//	std::cout << "evaluateIntermediate() = " 				            << ADcostFun->evaluateIntermediate() 			                << std::endl;
//	std::cout << "intermediateStateDerivative() = " 		<< ADcostFun->intermediateStateDerivative()  		<< std::endl;
//	std::cout << "intermediateStateSecondDerivative() = " 	<< ADcostFun->intermediateStateSecondDerivative()  	<< std::endl;
//	std::cout << "intermediateControlDerivative() = "		<< ADcostFun->intermediateControlDerivative() 		<< std::endl;
//	std::cout << "intermediateControlSecondDerivative() = " << ADcostFun->intermediateControlSecondDerivative() << std::endl;
//	std::cout << "intermediateStateControlDerivative() = " 	<< ADcostFun->intermediateStateControlDerivative()  << std::endl;
//
//	std::cout << "evaluateTerminal() = " 						<< ADcostFun->terminalCostCost() 					<< std::endl;
//	std::cout << "finalStateDerivative() = " 				<< ADcostFun->finalStateDerivative()  				<< std::endl;
//	std::cout << "finalStateSecondDerivative() = " 			<< ADcostFun->finalStateSecondDerivative()  		<< std::endl;
//	std::cout << "controlDerivativeTerminal() = "				<< ADcostFun->controlDerivativeTerminal() 				<< std::endl;
//	std::cout << "controlSecondDerivativeTerminal() = " 		<< ADcostFun->controlSecondDerivativeTerminal() 		<< std::endl;
//	std::cout << "stateControlDerivativeTerminal() = " 		<< ADcostFun->stateControlDerivativeTerminal()  		<< std::endl;

	return 0;
}
