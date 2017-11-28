/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::LQOCProblem(int N)
    : isConstrained_(false)  // by default, we assume the problem ins unconstrained
{
    changeNumStages(N);
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isConstrained() const
{
    return isConstrained_;
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

    u_lb_.resize(N);
    u_ub_.resize(N);
    x_lb_.resize(N + 1);
    x_ub_.resize(N + 1);

    d_lb_.resize(N + 1);
    d_ub_.resize(N + 1);
    C_.resize(N + 1);
    D_.resize(N + 1);
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setZero(const int& nGenConstr)
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


    // despite this method is called setZero() the upper and lower box constraints are set to the numeric limits
    core::ControlVector<CONTROL_DIM, SCALAR> umin, umax;
    umin.setConstant(std::numeric_limits<SCALAR>::lowest());
    umax.setConstant(std::numeric_limits<SCALAR>::max());
    u_lb_.setConstant(umin);
    u_ub_.setConstant(umax);

    core::StateVector<STATE_DIM, SCALAR> xmin, xmax;
    xmin.setConstant(std::numeric_limits<SCALAR>::lowest());
    xmax.setConstant(std::numeric_limits<SCALAR>::max());
    x_lb_.setConstant(xmin);
    x_ub_.setConstant(xmax);

    assert(d_lb_.size() == d_ub_.size());
    assert(d_lb_.size() == C_.size());
    assert(d_lb_.size() == D_.size());
    for (size_t i = 0; i < d_lb_.size(); i++)
    {
        d_lb_[i].resize(nGenConstr, 1);
        d_lb_[i].setZero();
        d_ub_[i].resize(nGenConstr, 1);
        d_ub_[i].setZero();
        C_[i].resize(nGenConstr, STATE_DIM);
        C_[i].setZero();
        D_[i].resize(nGenConstr, CONTROL_DIM);
        D_[i].setZero();
    }

    isConstrained_ = false;
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setStateBoxConstraints(ct::core::StateVector<STATE_DIM, SCALAR>& x_lb,
    ct::core::StateVector<STATE_DIM, SCALAR>& x_ub)
{
    x_lb_.setConstant(x_lb);
    x_ub_.setConstant(x_ub);
    isConstrained_ = true;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setControlBoxConstraints(
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_lb,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_ub)
{
    u_lb_.setConstant(u_lb);
    u_ub_.setConstant(u_ub);
    isConstrained_ = true;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setGeneralConstraints(constr_vec_t& d_lb,
    constr_vec_t& d_ub,
    constr_state_jac_t& C,
    constr_control_jac_t& D)
{
	d_lb_.setConstant(d_lb);
	d_ub_.setConstant(d_ub);
	C_.setConstant(C_);
	D_.setConstant(D_);
	isConstrained_ = true;
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

    isConstrained_ = false;
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setFromTimeInvariantLinearQuadraticProblem(
    ct::core::StateVector<STATE_DIM, SCALAR>& x0,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u0,
    ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
    ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
    ct::core::StateVector<STATE_DIM, SCALAR>& stateOffset,
    ct::core::StateVector<STATE_DIM, SCALAR>& x_lb,
    ct::core::StateVector<STATE_DIM, SCALAR>& x_ub,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_lb,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_ub,
    double dt)
{
    setFromTimeInvariantLinearQuadraticProblem(x0, u0, linearSystem, costFunction, stateOffset, dt);

    setControlBoxConstraints(u_lb, u_ub);
    setStateBoxConstraints(x_lb, x_ub);
}

}  // namespace optcon
}  // namespace ct
