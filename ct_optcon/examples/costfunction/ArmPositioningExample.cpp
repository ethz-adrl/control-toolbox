/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <ct/optcon/costfunction/CostFunctionAD.hpp>
#include <ct/optcon/costfunction/term/TermBase.hpp>

#include "../../examples/costfunction/EEDistanceTerm.h"

/*!
 * This example generates a cost function using the cartesian distance of an end-effector to a target position.
 */
int main()
{
    using namespace ct::core;
    using namespace ct::optcon::example;

    // autodiff costfunction
    std::shared_ptr<CostFunctionAD<stateDim_planar, controlDim_planar>> ADcostFun(
        new CostFunctionAD<stateDim_planar, controlDim_planar>());

    Eigen::Matrix3d Q_ee;
    Q_ee.setIdentity();
    Eigen::Matrix3d Q_ee_f;
    Q_ee_f.setIdentity();
    Q_ee_f *= 100;

    Eigen::Vector3d desiredEEPos;
    desiredEEPos << 1.0, 1.0, 1.0;

    std::shared_ptr<EEDistanceTerm> term1(new EEDistanceTerm(desiredEEPos, Q_ee));
    ADcostFun->addIntermediateTerm(term1);

    std::shared_ptr<EEDistanceTerm> term1_final(new EEDistanceTerm(desiredEEPos, Q_ee_f));
    ADcostFun->addFinalTerm(term1_final);


    Eigen::Vector2d x;
    Eigen::Matrix<double, controlDim_planar, 1> u;

    x.setRandom();
    u.setRandom();

    double t = 0.0;

    ADcostFun->setCurrentStateAndControl(x, u, t);

    std::cout << "evaluateIntermediate() = " << ADcostFun->evaluateIntermediate() << std::endl;
    std::cout << "intermediateStateDerivative() = " << ADcostFun->intermediateStateDerivative() << std::endl;
    std::cout << "intermediateStateSecondDerivative() = " << ADcostFun->intermediateStateSecondDerivative()
              << std::endl;
    std::cout << "intermediateControlDerivative() = " << ADcostFun->intermediateControlDerivative() << std::endl;
    std::cout << "intermediateControlSecondDerivative() = " << ADcostFun->intermediateControlSecondDerivative()
              << std::endl;
    std::cout << "intermediateStateControlDerivative() = " << ADcostFun->intermediateStateControlDerivative()
              << std::endl;

    std::cout << "evaluateTerminal() = " << ADcostFun->terminalCostCost() << std::endl;
    std::cout << "finalStateDerivative() = " << ADcostFun->finalStateDerivative() << std::endl;
    std::cout << "finalStateSecondDerivative() = " << ADcostFun->finalStateSecondDerivative() << std::endl;
    std::cout << "controlDerivativeTerminal() = " << ADcostFun->controlDerivativeTerminal() << std::endl;
    std::cout << "controlSecondDerivativeTerminal() = " << ADcostFun->controlSecondDerivativeTerminal() << std::endl;
    std::cout << "stateControlDerivativeTerminal() = " << ADcostFun->stateControlDerivativeTerminal() << std::endl;

    return 0;
}
