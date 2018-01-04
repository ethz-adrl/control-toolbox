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
    : hasBoxConstraints_(false),  // by default, we assume the problem ins unconstrained
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
bool LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::isBoxConstrained() const
{
    return hasBoxConstraints_;
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
    nb_.resize(N + 1);

    ng_.resize(N + 1);
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

    // reset the number of box constraints
    std::fill(nb_.begin(), nb_.end(), 0);

    assert(d_lb_.size() == d_ub_.size());
    assert(d_lb_.size() == C_.size());
    assert(d_lb_.size() == D_.size());
    std::fill(ng_.begin(), ng_.end(), 0);
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

    hasBoxConstraints_ = false;
    hasGenConstraints_ = false;
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setIntermediateBoxConstraint(const int index,
    const int nConstr,
    const constr_vec_t& ux_lb,
    const constr_vec_t& ux_ub,
    const VectorXi& sp)
{
    if ((ux_lb.rows() != ux_ub.rows()) | (ux_lb.size() != nConstr) | (sp.rows() != nConstr) |
        (sp(sp.rows() - 1) > (STATE_DIM + CONTROL_DIM - 1)))
    {
        std::cout << "n.o. constraints : " << nConstr << std::endl;
        std::cout << "ux_lb : " << ux_lb.transpose() << std::endl;
        std::cout << "ux_ub : " << ux_ub.transpose() << std::endl;
        std::cout << "sparsity : " << sp.transpose() << std::endl;
        throw(std::runtime_error("LQOCProblem setIntermediateBoxConstraint: error in constraint config"));
    }

    if (index >= K_)
        throw(std::runtime_error("LQOCProblem cannot set an intermediate Box constraint at time >= K_"));

    nb_[index] = nConstr;
    ux_lb_[index].template topRows(nConstr) = ux_lb;
    ux_ub_[index].template topRows(nConstr) = ux_ub;
    ux_I_[index].template topRows(nConstr) = sp;

    hasBoxConstraints_ = true;
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setIntermediateBoxConstraints(const int nConstr,
    const constr_vec_t& ux_lb,
    const constr_vec_t& ux_ub,
    const VectorXi& sp)
{
    for (int i = 0; i < K_; i++)
        setIntermediateBoxConstraint(i, nConstr, ux_lb, ux_ub, sp);
}


template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setTerminalBoxConstraints(const int nConstr,
    const constr_vec_t& ux_lb,
    const constr_vec_t& ux_ub,
    const VectorXi& sp)
{
    if (nConstr > 0)
    {
        if ((ux_lb.rows() != ux_ub.rows()) | (ux_lb.size() != nConstr) | (sp.rows() != nConstr) |
            (sp(sp.rows() - 1) > (STATE_DIM - 1)))
        {
            std::cout << "n.o. constraints : " << nConstr << std::endl;
            std::cout << "ux_lb : " << ux_lb.transpose() << std::endl;
            std::cout << "ux_ub : " << ux_ub.transpose() << std::endl;
            std::cout << "sparsity : " << sp.transpose() << std::endl;
            throw(std::runtime_error("LQOCProblem setTerminalBoxConstraint: error in constraint config"));
        }

        nb_[K_] = nConstr;
        ux_lb_[K_].template topRows(nConstr) = ux_lb;
        ux_ub_[K_].template topRows(nConstr) = ux_ub;
        ux_I_[K_].template topRows(nConstr) = sp;

        hasBoxConstraints_ = true;
    }
}

template <int STATE_DIM, int CONTROL_DIM, typename SCALAR>
void LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>::setGeneralConstraints(const constr_vec_t& d_lb,
    const constr_vec_t& d_ub,
    const constr_state_jac_t& C,
    const constr_control_jac_t& D)
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
    setZero();

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

    hasBoxConstraints_ = false;
    hasGenConstraints_ = false;
}

}  // namespace optcon
}  // namespace ct
