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
    : hasStateBoxConstraints_(false),  // by default, we assume the problem ins unconstrained
      hasControlBoxConstraints_(false),
      hasGenConstraints_(false)
{
    changeNumStages(N);
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isConstrained() const
{
    return (isBoxConstrained() | isGeneralConstrained());
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isControlBoxConstrained() const
{
    return hasControlBoxConstraints_;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isStateBoxConstrained() const
{
    return hasStateBoxConstraints_;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isBoxConstrained() const
{
    return hasStateBoxConstraints_ | hasControlBoxConstraints_;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isGeneralConstrained() const
{
    return hasGenConstraints_;
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

    ux_lb_.resize(N + 1);
    ux_ub_.resize(N + 1);
    ux_I_.resize(N + 1);

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
    box_constr_t uxmin, uxmax;
    uxmin.setConstant(std::numeric_limits<SCALAR>::lowest());
    uxmax.setConstant(std::numeric_limits<SCALAR>::max());
    ux_lb_.setConstant(uxmin);
    ux_ub_.setConstant(uxmax);
    ux_I_.setConstant(box_constr_sparsity_t::Zero());

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

    hasStateBoxConstraints_ = false;
    hasControlBoxConstraints_ = false;
    hasGenConstraints_ = false;
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setStateBoxConstraints(ct::core::StateVector<STATE_DIM, SCALAR>& x_lb,
    ct::core::StateVector<STATE_DIM, SCALAR>& x_ub,
    const Eigen::Matrix<int, STATE_DIM, 1>& sparsity)
{
    // make sure there are no state bounds at initial stage
    ux_lb_[0].template tail<STATE_DIM>().setConstant(std::numeric_limits<SCALAR>::lowest());
    ux_ub_[0].template tail<STATE_DIM>().setConstant(std::numeric_limits<SCALAR>::max());
    ux_I_[0].template tail<STATE_DIM>() = Eigen::Matrix<int, STATE_DIM, 1>::Zero();

    assert(ux_lb_.size() == ux_ub_.size());
    for (size_t i = 1; i < K_ + 1; i++)
    {
        ux_lb_[i].template tail<STATE_DIM>() = x_lb;
        ux_ub_[i].template tail<STATE_DIM>() = x_ub;
        ux_I_[i].template tail<STATE_DIM>() = sparsity;
    }

    hasStateBoxConstraints_ = true;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setControlBoxConstraints(
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_lb,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_ub,
    const Eigen::Matrix<int, CONTROL_DIM, 1>& sparsity)
{
    assert(ux_lb_.size() == ux_ub_.size());
    for (size_t i = 0; i < K_; i++)
    {
        ux_lb_[i].template head<CONTROL_DIM>() = u_lb;
        ux_ub_[i].template head<CONTROL_DIM>() = u_ub;
        ux_I_[i].template head<CONTROL_DIM>() = sparsity;
    }

    // make sure there are no control bounds at terminal stage
    ux_lb_[K_].template head<CONTROL_DIM>().setConstant(std::numeric_limits<SCALAR>::lowest());
    ux_ub_[K_].template head<CONTROL_DIM>().setConstant(std::numeric_limits<SCALAR>::max());
    ux_I_[K_].template head<CONTROL_DIM>() = Eigen::Matrix<int, CONTROL_DIM, 1>::Zero();

    hasControlBoxConstraints_ = true;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setGeneralConstraints(constr_vec_t& d_lb,
    constr_vec_t& d_ub,
    constr_state_jac_t& C,
    constr_control_jac_t& D)
{
    d_lb_.setConstant(d_lb);
    d_ub_.setConstant(d_ub);
    C_.setConstant(C);
    D_.setConstant(D);
    hasGenConstraints_ = true;
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

    hasStateBoxConstraints_ = false;
    hasControlBoxConstraints_ = false;
    hasGenConstraints_ = false;
}

}  // namespace optcon
}  // namespace ct
