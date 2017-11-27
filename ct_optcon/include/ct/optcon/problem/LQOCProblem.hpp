/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/*!
 *
 * This class defines a Linear Quadratic Optimal Control (LQOC) Problem, consisting of
 * - affine systen dynamics
 * - reference trajectories (arrays!) for state and control
 * - LQ approximation of the cost function
 *
 * The unconstrained LQ problem hence has the following form:
 * \f[
 * \min_{\delta \mathbf{u}_n, \delta \mathbf{x}_n}
 * \bigg \{
 * q_N +\delta \mathbf{x}_N^\top \mathbf{q}_N +\frac{1}{2}\delta \mathbf{x}_N^\top \mathbf{Q}_N\delta \mathbf{x}_N
 * +\sum_{n=0}^{N-1} q_n + \delta \mathbf{x}_n^\top \mathbf{q}_n
 * + \delta \mathbf{u}_n^\top \mathbf{r}_n
 * + \frac{1}{2}\delta \mathbf{x}_n^\top\mathbf{Q}_n\delta \mathbf{x}_n
 * +\frac{1}{2}\delta \mathbf{u}_n^\top \mathbf{R}_n\delta \mathbf{u}_n
 * + \delta \mathbf{u}_n^\top \mathbf{P}_n\delta \mathbf{x}_n
 * \bigg \}
 * \f]
 * subject to
 * \f[
 * \delta \mathbf x_{n+1} = \mathbf A_n \delta \mathbf x_n + \mathbf B_n \delta \mathbf u_n +\mathbf b_n
 * \f]
 * with
 * \f$ \delta \mathbf x_n = \mathbf x_n - \hat \mathbf x_n \f$ and \f$ \delta \mathbf u_n = \mathbf u_n - \hat \mathbf u_n \f$
 *
 * The reference trajectories for state and control are here denoted as \f$ \hat \mathbf x_i, \
 *  \hat \mathbf u_i \quad \forall i = 0, 1, \ldots \f$
 *
 * The constrained LQ problem additionally implements the box constraints
 * \f$ \mathbf \x_{lb} \leq \mathbf x_n \leq \mathbf \x_{ub} \ \forall i=1,2,\ldots,N \f$ and
 * \f$ \mathbf \u_{lb} \leq \mathbf u_n \leq \mathbf \u_{ub} \ \forall i=0,1,\ldots,N-1  \f$
 * which in differential notation read as
 * \f[
 * \mathbf \x_{lb} - \hat \mathbf x_n  \leq \delta \mathbf x_n \leq \mathbf \x_{ub} - \hat \mathbf x_n \ \forall i=1,2,\ldots,N
 * \f]
 * and
 * \f[
 * \mathbf \u_{lb} - \hat \mathbf u_n  \leq \delta \mathbf u_n \leq \mathbf \u_{ub} - \hat \mathbf u_n \ \forall i=0,1,\ldots,N-1
 * \f]
 * and the general inequality constraints
 * \f[
 * \mathbf \d_{lb}  \leq \mathbf C_n \delta \mathbf \x_n + \mathbf D_n \delta \mathbf \u_n \leq \mathbf \d_{ub} \ \forall i=0,1,\ldots,N
 * \f]
 */
template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class LQOCProblem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using constr_vec_t = Eigen::Matrix<SCALAR, -1, 1>;
    using constr_state_jac_t = Eigen::Matrix<SCALAR, -1, STATE_DIM>;
    using constr_control_jac_t = Eigen::Matrix<SCALAR, -1, CONTROL_DIM>;

    using constr_vec_array_t = ct::core::DiscreteArray<constr_vec_t>;
    using constr_state_jac_array_t =  ct::core::DiscreteArray<constr_state_jac_t>;
    using constr_control_jac_array_t = ct::core::DiscreteArray<constr_control_jac_t>;


    //! constructor
    LQOCProblem(int N = 0);

    //! returns the number of discrete time steps in the LOCP, including terminal stage
    int getNumberOfStages();

    //! change the number of discrete time steps in the LOCP
    void changeNumStages(int N);

    /*!
     * @brief set all member variables to zero
     * @param nGenConstr by default, we resize the general constraint containers to zero
     */
    void setZero(const int& nGenConstr = 0);

    //! set state box constraints
    void setStateBoxConstraints(ct::core::StateVector<STATE_DIM, SCALAR>& x_lb,
        ct::core::StateVector<STATE_DIM, SCALAR>& x_ub);

    //! set control box constraints
    void setControlBoxConstraints(ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_lb,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_ub);

    /*!
     * \brief a convenience method which constructs an unconstrained LQOC Problem from an LTI system and continuous-time quadratic cost
     * The discretization of the cost functions happens within this function. It employs an Euler-Discretization
     * @param x0 the initial state
     * @param u0 the current (and desired control)
     * @param linearSystem the discrete-time LTI system
     * @param costFunction the continuous-time cost function
     * @param stateOffset the offset for the affine system dynamics demanded by the LQOC Solver
     * @param dt the sampling time, required for discretization
     */
    void setFromTimeInvariantLinearQuadraticProblem(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& u0,
        ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
        ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
        ct::core::StateVector<STATE_DIM, SCALAR>& stateOffset,
        double dt);

    /*!
     * \brief a convenience method which constructs a box-constrained LQOC Problem from an LTI system and continuous-time quadratic cost
     * The discretization of the cost functions and constraints happens within this function. It employs an Euler-Discretization
     * @param x0 the initial state
     * @param u0 the current (and desired control)
     * @param linearSystem the discrete-time LTI system
     * @param costFunction the continuous-time cost function
     * @param stateOffset the offset for the affine system dynamics demanded by the LQOC Solver
     * @param dt the sampling time, required for discretization
     * @param x_lb state lower bound
     * @param x_ub state upper bound
     * @param u_lb control lower bound
     * @param u_ub control upper bound
     */
    void setFromTimeInvariantLinearQuadraticProblem(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& u0,
        ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
        ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
        ct::core::StateVector<STATE_DIM, SCALAR>& stateOffset,
        ct::core::StateVector<STATE_DIM, SCALAR>& x_lb,
        ct::core::StateVector<STATE_DIM, SCALAR>& x_ub,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_lb,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_ub,
        double dt);

    //! return a flag indicating whether this LQOC Problem is constrained or not
    bool isConstrained() const;

    //! affine, time-varying system dynamics in discrete time
    ct::core::StateMatrixArray<STATE_DIM, SCALAR> A_;
    ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> B_;
    ct::core::StateVectorArray<STATE_DIM, SCALAR> b_;

    //! reference state trajectory
    ct::core::StateVectorArray<STATE_DIM, SCALAR> x_;

    //! reference control trajectory
    ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_;

    //! constant term of in the LQ approximation of the cost function
    ct::core::ScalarArray<SCALAR> q_;

    //! LQ approximation of the pure state penalty, including terminal state penalty
    ct::core::StateVectorArray<STATE_DIM, SCALAR> qv_;
    ct::core::StateMatrixArray<STATE_DIM, SCALAR> Q_;

    //! LQ approximation of the pure control penalty
    ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> rv_;
    ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> R_;

    //! LQ approximation of the cross terms of the cost function
    ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> P_;


    //! lower bound of control input box constraint
    ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_lb_;
    //! upper bound of control input box constraint
    ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_ub_;

    //! lower bound of state box constraint
    ct::core::StateVectorArray<STATE_DIM, SCALAR> x_lb_;
    //! upper bound of state box constraint
    ct::core::StateVectorArray<STATE_DIM, SCALAR> x_ub_;

    //! general constraint lower bound
    constr_vec_array_t d_lb_;
    //! general constraint upper bound
    constr_vec_array_t d_ub_;

    //! linear general constraint matrices
    constr_state_jac_array_t C_;
    constr_control_jac_array_t D_;

private:
    //! the number of discrete time steps in the LOCP, including terminal stage
    int K_;

    //! bool indicating if the optimization problem is constrained
    bool isConstrained_;
};

}  // namespace optcon
}  // namespace ct
