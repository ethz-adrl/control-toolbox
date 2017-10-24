/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::LQOCProblem(int N)
{
    changeNumStages(N);
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
int LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::getNumberOfStages()
{
    return K_;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::changeNumStages(int N)
{
    K_ = N;

    A_.resize(N);
    B_.resize(N);
    b_.resize(N + 1);

    x_.resize(N + 1);
    u_.resize(N);

    P_.resize(N);
    q_.resize(N + 1);
    qv_.resize(N + 1);
    Q_.resize(N + 1);

    rv_.resize(N);
    R_.resize(N);
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setZero()
{
    A_.setConstant(core::StateMatrix<STATE_DIM, SCALAR>::Zero());
    B_.setConstant(core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR>::Zero());
    b_.setConstant(core::StateVector<STATE_DIM, SCALAR>::Zero());
    x_.setConstant(core::StateVector<STATE_DIM, SCALAR>::Zero());
    u_.setConstant(core::ControlVector<CONTROL_DIM, SCALAR>::Zero());
    P_.setConstant(core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>::Zero());
    qv_.setConstant(core::StateVector<STATE_DIM, SCALAR>::Zero());
    Q_.setConstant(core::StateMatrix<STATE_DIM, SCALAR>::Zero());
    rv_.setConstant(core::ControlVector<CONTROL_DIM, SCALAR>::Zero());
    R_.setConstant(core::ControlMatrix<CONTROL_DIM, SCALAR>::Zero());
    q_.setConstant((SCALAR)0.0);
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setFromTimeInvariantLinearQuadraticProblem(
    ct::core::StateVector<STATE_DIM, SCALAR>& x0,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u0,
    ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
    ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
    ct::core::StateVector<STATE_DIM, SCALAR>& stateOffset,
    double dt)
{
    core::StateMatrix<STATE_DIM, SCALAR> A;
    core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> B;
    linearSystem.getAandB(x0, u0, 0, A, B);

    A_ = core::StateMatrixArray<STATE_DIM, SCALAR>(K_, A);
    B_ = core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR>(K_, B);
    b_ = core::StateVectorArray<STATE_DIM, SCALAR>(K_ + 1, stateOffset);


    // feed current state and control to cost function
    costFunction.setCurrentStateAndControl(x0, u0, 0);

    // derivative of cost with respect to state
    qv_ = core::StateVectorArray<STATE_DIM, SCALAR>(K_ + 1, costFunction.stateDerivativeIntermediate() * dt);
    Q_ = core::StateMatrixArray<STATE_DIM, SCALAR>(K_ + 1, costFunction.stateSecondDerivativeIntermediate() * dt);

    // derivative of cost with respect to control and state
    P_ =
        core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>(K_, costFunction.stateControlDerivativeIntermediate() * dt);

    // derivative of cost with respect to control
    rv_ = core::ControlVectorArray<CONTROL_DIM, SCALAR>(K_, costFunction.controlDerivativeIntermediate() * dt);

    R_ = core::ControlMatrixArray<CONTROL_DIM, SCALAR>(K_, costFunction.controlSecondDerivativeIntermediate() * dt);

    Q_[K_] = costFunction.stateSecondDerivativeTerminal();
    qv_[K_] = costFunction.stateDerivativeTerminal();

    x_ = core::StateVectorArray<STATE_DIM, SCALAR>(K_ + 1, x0);
    u_ = core::ControlVectorArray<CONTROL_DIM, SCALAR>(K_, u0);
}

}  //! optcon
}  //! ct
