/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename MANIFOLD, int CONTROL_DIM>
LQOCProblem<MANIFOLD, CONTROL_DIM>::LQOCProblem(int N) : nbu_(N, 0), nbx_(N + 1, 0)
{
    changeNumStages(N);
}

template <typename MANIFOLD, int CONTROL_DIM>
LQOCProblem<MANIFOLD, CONTROL_DIM>::~LQOCProblem()
{
}

template <typename MANIFOLD, int CONTROL_DIM>
bool LQOCProblem<MANIFOLD, CONTROL_DIM>::isConstrained() const
{
    return (isInputBoxConstrained() | isStateBoxConstrained() | isGeneralConstrained());
}
template <typename MANIFOLD, int CONTROL_DIM>
bool LQOCProblem<MANIFOLD, CONTROL_DIM>::isInputBoxConstrained() const
{
    if (std::accumulate(nbu_.begin(), nbu_.end(), 0) > 0)
        return true;

    return false;
}

template <typename MANIFOLD, int CONTROL_DIM>
bool LQOCProblem<MANIFOLD, CONTROL_DIM>::isStateBoxConstrained() const
{
    if (std::accumulate(nbx_.begin(), nbx_.end(), 0) > 0)
        return true;

    return false;
}

template <typename MANIFOLD, int CONTROL_DIM>
bool LQOCProblem<MANIFOLD, CONTROL_DIM>::isGeneralConstrained() const
{
    if (std::accumulate(ng_.begin(), ng_.end(), 0) > 0)
        return true;

    return false;
}

template <typename MANIFOLD, int CONTROL_DIM>
int LQOCProblem<MANIFOLD, CONTROL_DIM>::getNumberOfStages()
{
    return K_;
}

template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::changeNumStages(int N)
{
    K_ = N;

    A_.resize(N);
    B_.resize(N);
    b_.resize(N + 1);

    P_.resize(N);
    q_.resize(N + 1);
    qv_.resize(N + 1);
    Q_.resize(N + 1);
    Adj_x_.resize(N + 1);

    rv_.resize(N);
    R_.resize(N);

    x_lb_.resize(N + 1);
    x_ub_.resize(N + 1);
    x_I_.resize(N + 1);
    u_lb_.resize(N);
    u_ub_.resize(N);
    u_I_.resize(N);

    nbx_.resize(N + 1);
    nbu_.resize(N);

    ng_.resize(N + 1);
    d_lb_.resize(N + 1);
    d_ub_.resize(N + 1);
    C_.resize(N + 1);
    D_.resize(N + 1);
}


template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setZero(const int& nGenConstr)
{
    A_.setConstant(core::StateMatrix<STATE_DIM, SCALAR>::Zero());
    B_.setConstant(core::StateControlMatrix<STATE_DIM, SCALAR>::Zero(STATE_DIM, CONTROL_DIM));
    b_.setConstant(core::StateVector<STATE_DIM, SCALAR>::Zero());
    P_.setConstant(core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>::Zero(CONTROL_DIM, STATE_DIM));
    qv_.setConstant(core::StateVector<STATE_DIM, SCALAR>::Zero());
    Q_.setConstant(core::StateMatrix<STATE_DIM, SCALAR>::Zero());
    Adj_x_.setConstant(core::StateMatrix<STATE_DIM, SCALAR>::Identity());
    rv_.setConstant(core::ControlVector<SCALAR>::Zero(CONTROL_DIM));
    R_.setConstant(core::ControlMatrix<SCALAR>::Zero(CONTROL_DIM, CONTROL_DIM));
    q_.setConstant((SCALAR)0.0);

    // reset the number of box constraints
    std::fill(nbx_.begin(), nbx_.end(), 0);
    std::fill(nbu_.begin(), nbu_.end(), 0);

    // reset general constraints
    assert(ng_.size() == d_lb_.size());
    assert(d_lb_.size() == d_ub_.size());
    assert(d_lb_.size() == C_.size());
    assert(d_lb_.size() == D_.size());
    std::fill(ng_.begin(), ng_.end(), nGenConstr);
    for (size_t i = 0; i < ng_.size(); i++)
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
}


template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setInputBoxConstraint(const int index,
    const int nConstr,
    const constr_vec_t& u_lb,
    const constr_vec_t& u_ub,
    const VectorXi& sp,
    const ct::core::ControlVector<SCALAR>& u_nom_abs)
{
    if ((u_lb.rows() != u_ub.rows()) || (u_lb.size() != nConstr) || (sp.rows() != nConstr) ||
        (sp(sp.rows() - 1) > (CONTROL_DIM - 1)))
    {
        std::cout << "n.o. constraints : " << nConstr << std::endl;
        std::cout << "u_lb : " << u_lb.transpose() << std::endl;
        std::cout << "u_ub : " << u_ub.transpose() << std::endl;
        std::cout << "sparsity : " << sp.transpose() << std::endl;
        throw(std::runtime_error("LQOCProblem setInputBoxConstraint: error in constraint config"));
    }

    if (index >= K_)
        throw(std::runtime_error("LQOCProblem cannot set an intermediate input Box constraint at time >= K_"));

    nbu_[index] = nConstr;

    // loop through box constraints and assign bounds in differential format
    for (int i = 0; i < nConstr; i++)
    {
        u_I_[index](i) = sp(i);
        u_lb_[index](i) = u_lb(i) - u_nom_abs(sp(i));  // substract the corresponding entry in nom-control
        u_ub_[index](i) = u_ub(i) - u_nom_abs(sp(i));  // substract the corresponding entry in nom-control
    }
}

template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setInputBoxConstraints(const int nConstr,
    const constr_vec_t& u_lb,
    const constr_vec_t& u_ub,
    const VectorXi& sp,
    const ct::core::ControlVectorArray<SCALAR>& u_nom_abs)
{
    for (int i = 0; i < K_; i++)
        setInputBoxConstraint(i, nConstr, u_lb, u_ub, sp, u_nom_abs[i]);
}


template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setIntermediateStateBoxConstraint(const int index,
    const int nConstr,
    const constr_vec_t& x_lb,
    const constr_vec_t& x_ub,
    const VectorXi& sp,
    const ct::core::StateVector<STATE_DIM, SCALAR>& x_nom_abs)
{
    if ((x_lb.rows() != x_ub.rows()) || (x_lb.size() != nConstr) || (sp.rows() != nConstr) ||
        (sp(sp.rows() - 1) > (STATE_DIM - 1)))
    {
        std::cout << "n.o. constraints : " << nConstr << std::endl;
        std::cout << "x_lb : " << x_lb.transpose() << std::endl;
        std::cout << "x_ub : " << x_ub.transpose() << std::endl;
        std::cout << "sparsity : " << sp.transpose() << std::endl;
        throw(std::runtime_error("LQOCProblem setIntermediateStateBoxConstraint: error in constraint config"));
    }

    if (index >= K_)
        throw(std::runtime_error("LQOCProblem cannot set an intermediate state Box constraint at time >= K_"));

    nbx_[index] = nConstr;

    // loop through box constraints and assign bounds in differential format
    for (int i = 0; i < nConstr; i++)
    {
        x_I_[index](i) = sp(i);
        x_lb_[index](i) = x_lb(i) - x_nom_abs(sp(i));  // substract the corresponding entry in nom-state
        x_ub_[index](i) = x_ub(i) - x_nom_abs(sp(i));  // substract the corresponding entry in nom-state
    }
}

template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setIntermediateStateBoxConstraints(const int nConstr,
    const constr_vec_t& x_lb,
    const constr_vec_t& x_ub,
    const VectorXi& sp,
    const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_nom_abs)
{
    for (int i = 0; i < K_; i++)
        setIntermediateStateBoxConstraint(i, nConstr, x_lb, x_ub, sp, x_nom_abs[i]);
}


template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setTerminalBoxConstraints(const int nConstr,
    const constr_vec_t& x_lb,
    const constr_vec_t& x_ub,
    const VectorXi& sp,
    const ct::core::StateVector<STATE_DIM, SCALAR>& x_nom_abs)
{
    if (nConstr > 0)
    {
        if ((x_lb.rows() != x_ub.rows()) || (x_lb.size() != nConstr) || (sp.rows() != nConstr) ||
            (sp(sp.rows() - 1) > (STATE_DIM - 1)))
        {
            std::cout << "n.o. constraints : " << nConstr << std::endl;
            std::cout << "ux_lb : " << x_lb.transpose() << std::endl;
            std::cout << "ux_ub : " << x_ub.transpose() << std::endl;
            std::cout << "sparsity : " << sp.transpose() << std::endl;
            throw(std::runtime_error("LQOCProblem setTerminalBoxConstraint: error in constraint config"));
        }

        nbx_[K_] = nConstr;

        // loop through box constraints and assign bounds in differential format
        for (int i = 0; i < nConstr; i++)
        {
            x_I_[K_](i) = sp(i);
            x_lb_[K_](i) = x_lb(i) - x_nom_abs(sp(i));  // substract the corresponding entry in nom-state
            x_ub_[K_](i) = x_ub(i) - x_nom_abs(sp(i));  // substract the corresponding entry in nom-state
        }
    }
}

template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setGeneralConstraints(const constr_vec_t& d_lb,
    const constr_vec_t& d_ub,
    const constr_state_jac_t& C,
    const constr_control_jac_t& D)
{
    if (d_lb.size() != d_ub.size() || C.rows() != D.rows() || d_lb.size() != C.rows())
    {
        std::cout << "d_lb : " << std::endl << d_lb << std::endl;
        std::cout << "d_ub : " << std::endl << d_ub << std::endl;
        std::cout << "C : " << std::endl << C << std::endl;
        std::cout << "D : " << std::endl << D << std::endl;
        throw(std::runtime_error("LQOCProblem setGeneralConstraints: error in constraint config"));
    }

    std::fill(ng_.begin(), ng_.end(), d_lb.rows());
    d_lb_.setConstant(d_lb);
    d_ub_.setConstant(d_ub);
    C_.setConstant(C);
    D_.setConstant(D);
}

template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setFromTimeInvariantLinearQuadraticProblem(const MANIFOLD& x0,
    const core::ControlVector<SCALAR>& u0,
    ct::core::LinearSystem<MANIFOLD, core::DISCRETE_TIME>& linearSystem,
    ct::optcon::CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>& costFunction,
    const typename MANIFOLD::Tangent& b,
    const double dt)
{
    setZero();

    core::StateMatrix<STATE_DIM, SCALAR> A;
    core::StateControlMatrix<STATE_DIM, SCALAR> B;
    linearSystem.getDerivatives(A, B, x0, u0, 0);

    A_ = core::StateMatrixArray<STATE_DIM, SCALAR>(K_, A);
    B_ = core::StateControlMatrixArray<STATE_DIM, SCALAR>(K_, B);
    b_ = core::DiscreteArray<typename MANIFOLD::Tangent>(K_ + 1, b);

    // feed current state and control to cost function
    costFunction.setCurrentStateAndControl(x0, u0, 0);

    // intermediate stage
    Q_ = core::StateMatrixArray<STATE_DIM, SCALAR>(K_ + 1, costFunction.stateSecondDerivativeIntermediate() * dt);
    P_ =
        core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>(K_, costFunction.stateControlDerivativeIntermediate() * dt);
    R_ = core::ControlMatrixArray<SCALAR>(K_, costFunction.controlSecondDerivativeIntermediate() * dt);

    qv_ = core::DiscreteArray<typename MANIFOLD::Tangent>(K_ + 1, costFunction.stateDerivativeIntermediate() * dt);
    rv_ = core::ControlVectorArray<SCALAR>(K_, costFunction.controlDerivativeIntermediate() * dt);

    // final stage
    Q_[K_] = costFunction.stateSecondDerivativeTerminal() * dt;
    qv_[K_] = costFunction.stateDerivativeTerminal() * dt;
}


template <typename MANIFOLD, int CONTROL_DIM>
void LQOCProblem<MANIFOLD, CONTROL_DIM>::setFromTimeInvariantLinearQuadraticProblem(
    const ct::core::DiscreteArray<MANIFOLD>& x_traj,
    const ct::core::DiscreteArray<core::ControlVector<SCALAR>>& u_traj,
    ct::core::LinearSystem<MANIFOLD, core::DISCRETE_TIME>& linearSystem,
    ct::optcon::CostFunctionQuadratic<MANIFOLD, CONTROL_DIM>& costFunction,
    const ct::core::DiscreteArray<typename MANIFOLD::Tangent>& b,
    const double dt)
{
    setZero();

    if ((int)b.size() != K_ + 1)
        throw std::runtime_error("b size not correct.");

    for (int i = 0; i < K_; i++)
    {
        // get LTI dynamics
        core::StateMatrix<STATE_DIM, SCALAR> A;
        core::StateControlMatrix<STATE_DIM, SCALAR> B;
        linearSystem.SetLinearizationPoint(x_traj[i], u_traj[i]);
        // get derivatives expressed at x, u setpoints.
        linearSystem.getDerivatives(A_[i], B_[i], 0);
    }
    b_ = b;  // TODO: this is wrong.

    // eval costs around the reference trajectory
    for (int i = 0; i < K_; i++)
    {
        // feed current state and control to cost function
        costFunction.setCurrentStateAndControl(x_traj[i], u_traj[i], i * dt);

        // intermediate stage
        Q_[i] = costFunction.stateSecondDerivativeIntermediate() * dt;
        P_[i] = costFunction.stateControlDerivativeIntermediate() * dt;
        R_[i] = costFunction.controlSecondDerivativeIntermediate() * dt;

        qv_[i] = costFunction.stateDerivativeIntermediate() * dt;
        rv_[i] = costFunction.controlDerivativeIntermediate() * dt;
    }
    // final stage
    costFunction.setCurrentStateAndControl(x_traj.back(), u_traj.back(), K_ * dt);
    Q_[K_] = costFunction.stateSecondDerivativeTerminal() * dt;
    qv_[K_] = costFunction.stateDerivativeTerminal() * dt;
}

}  // namespace optcon
}  // namespace ct
