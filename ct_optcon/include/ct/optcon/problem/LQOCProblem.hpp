/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

/*!
 * \brief Defines a Linear-Quadratic Optimal Control Problem, which is optionally constrained.
 *
 * This class defines a Linear Quadratic Optimal Control (LQOC) Problem in differential formulation, consisting of
 * - affine system dynamics
 * - LQ approximation of the cost function
 *
 * The unconstrained LQ problem has the following form:
 * \f[
 * \min_{\delta \mathbf{u}_n, \delta \mathbf{x}_n}
 * \bigg \{
 * q_N + \delta \mathbf{x}_N^\top \mathbf{q}_N +\frac{1}{2} \delta \mathbf{x}_N^\top \mathbf{Q}_N  \delta \mathbf{x}_N
 * +\sum_{n=0}^{N-1} q_n + \delta \mathbf{x}_n^\top \mathbf{q}_n
 * + \delta \mathbf{u}_n^\top \mathbf{r}_n
 * + \frac{1}{2} \delta \mathbf{x}_n^\top\mathbf{Q}_n \delta \mathbf{x}_n
 * +\frac{1}{2} \delta \mathbf{u}_n^\top \mathbf{R}_n \delta \mathbf{u}_n
 * + \delta \mathbf{u}_n^\top \mathbf{P}_n \delta \mathbf{x}_n
 * \bigg \}
 * \f]
 * subject to
 * \f[
 * \mathbf \delta x_{n+1} = \mathbf A_n \delta \mathbf x_n + \mathbf B_n \delta \mathbf u_n +\mathbf b_n
 * \f]
 *
 * The constrained LQ problem additionally implements the box constraints
 * \f$ \mathbf x_{lb} \leq \mathbf \delta x_n \leq \mathbf x_{ub} \ \forall i=1,2,\ldots,N \f$
 * and
 * \f$ \mathbf u_{lb} \leq \delta \mathbf u_n \leq \mathbf u_{ub} \ \forall i=0,1,\ldots,N-1\f$
 * and the general inequality constraints
 * \f[
 * \mathbf d_{lb}  \leq \mathbf C_n \delta \mathbf x_n + \mathbf D_n \delta \mathbf u_n \leq \mathbf d_{ub} \ \forall i=0,1,\ldots,N
 * \f]
 * which are both always kept in relative coordinates.
 *
 * \note The box constraint containers within this class are made fixed-size. Solvers can get the
 * actual number of box constraints from a a dedicated container nbu_ and nbx_
 *
 * \note In the differential notation we define
 * \f$ \delta \mathbf x_n = \mathbf x_n - \hat \mathbf x_n \f$ and \f$ \delta \mathbf u_n = \mathbf u_n - \hat \mathbf u_n \f$
 * where  \hat \mathbf x_n and  \hat \mathbf u_n are current nominal/reference trajectories, around which the LQP is formed.
 *
 * \note this problem does not contain any state or control trajectory! The fundamental assumption is that \f$ \delta \mathbf x_0 = 0 \f$
 *
 * \todo Refactor the initializing methods such that const-references can be handed over.
 */
template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class LQOCProblem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using constr_vec_t = Eigen::Matrix<SCALAR, -1, -1>;
    using constr_state_jac_t = Eigen::Matrix<SCALAR, -1, -1>;
    using constr_control_jac_t = Eigen::Matrix<SCALAR, -1, -1>;

    using constr_vec_array_t = ct::core::DiscreteArray<constr_vec_t>;
    using constr_state_jac_array_t = ct::core::DiscreteArray<constr_state_jac_t>;
    using constr_control_jac_array_t = ct::core::DiscreteArray<constr_control_jac_t>;

    using input_box_constr_vec_t = Eigen::Matrix<SCALAR, CONTROL_DIM, 1>;
    using state_box_constr_vec_t = Eigen::Matrix<SCALAR, STATE_DIM, 1>;
    using input_box_constr_array_t = ct::core::DiscreteArray<input_box_constr_vec_t>;
    using state_box_constr_array_t = ct::core::DiscreteArray<state_box_constr_vec_t>;

    //! a vector indicating which box constraints are active and which not
    using state_box_constr_sparsity_t = Eigen::Matrix<int, STATE_DIM, 1>;
    using input_box_constr_sparsity_t = Eigen::Matrix<int, CONTROL_DIM, 1>;

    using input_box_constr_sparsity_array_t = ct::core::DiscreteArray<input_box_constr_sparsity_t>;
    using state_box_constr_sparsity_array_t = ct::core::DiscreteArray<state_box_constr_sparsity_t>;

    using VectorXi = Eigen::VectorXi;

    //! constructor
    LQOCProblem(int N = 0);

    //! returns the number of discrete time steps in the LQOCP, including terminal stage
    int getNumberOfStages();

    //! change the number of discrete time steps in the LQOCP
    void changeNumStages(int N);

    /*!
     * @brief set all member variables to zero
     * @param nGenConstr by default, we resize the general constraint containers to zero
     */
    void setZero(const int& nGenConstr = 0);

    /*!
     * \brief set input box constraints at a specific index
     * @param index the index
     * @param nConstr the number of constraints
     * @param u_lb control lower bound in absolute coordinates
     * @param u_ub control upper bound in absolute coordinates
     * @param sp the sparsity vector, with strictly increasing indices, e.g. [0 1 4 7]
     */
    void setInputBoxConstraint(const int index,
        const int nConstr,
        const constr_vec_t& u_lb,
        const constr_vec_t& u_ub,
        const VectorXi& sp,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u_nom_abs);

    /*!
     * \brief set uniform input box constraints, with the same constraint being applied at each intermediate stage
     * @param nConstr the number of constraints
     * @param u_lb control lower bound in absolute coordinates
     * @param u_ub control upper bound in absolute coordinates
     * @param sp the sparsity vector, with strictly increasing indices, e.g. [0 1 4 7]
     * @param u_nom_abs current nominal control trajectory in absolute coordinates, required for coordinate transform
     */
    void setInputBoxConstraints(const int nConstr,
        const constr_vec_t& u_lb,
        const constr_vec_t& u_ub,
        const VectorXi& sp,
        const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_nom_abs);

    /*!
     * \brief set state box constraints at a specific index
     * @param index the index
     * @param nConstr the number of constraints
     * @param x_lb state lower bound in absolute coordinates
     * @param x_ub state upper bound in absolute coordinates
     * @param sp the sparsity vector, with strictly increasing indices, e.g. [0 1 4 7]
     * @param x_nom_abs current nominal state vector in absolute coordinates, required for coordinate transform
     */
    void setIntermediateStateBoxConstraint(const int index,
        const int nConstr,
        const constr_vec_t& x_lb,
        const constr_vec_t& x_ub,
        const VectorXi& sp,
        const ct::core::StateVector<STATE_DIM, SCALAR>& x_nom_abs);

    /*!
     * \brief set uniform state box constraints, with the same constraint being applied at each intermediate stage
     * @param nConstr the number of constraints
     * @param x_lb control lower bound in absolute coordinates
     * @param x_ub control upper bound in absolute coordinates
     * @param sp the sparsity vector, with strictly increasing indices, e.g. [0 1 4 7]
     * @param x_nom_abs current nominal state trajectory in absolute coordinates, required for coordinate transform
     */
    void setIntermediateStateBoxConstraints(const int nConstr,
        const constr_vec_t& x_lb,
        const constr_vec_t& x_ub,
        const VectorXi& sp,
        const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_nom_abs);

    /*!
     * \brief set box constraints for terminal stage
     * @param nConstr the number of constraints
     * @param x_lb state lower bound in absolute coordinates
     * @param x_ub state upper bound in absolute coordinates
     * @param sp the sparsity vector, with strictly increasing indices, e.g. [0 1 4 7]
     * @param x_nom_abs current nominal terminal state vector in absolute coordinates, required for coordinate transform
     */
    void setTerminalBoxConstraints(const int nConstr,
        const constr_vec_t& x_lb,
        const constr_vec_t& x_ub,
        const VectorXi& sp,
        const ct::core::StateVector<STATE_DIM, SCALAR>& x_nom_abs);

    /*!
     * \brief set general (in)equaltiy constraints, with the same constraint applied at each stage
     * @param d_lb general constraint lower bound
     * @param d_ub general constraint upper bound
     * @param C general constraint state jacobian
     * @param D general constraint control jacobian
     */
    void setGeneralConstraints(const constr_vec_t& d_lb,
        const constr_vec_t& d_ub,
        const constr_state_jac_t& C,
        const constr_control_jac_t& D);

    /*!
     * \brief a convenience method which constructs an unconstrained LQOC Problem from an LTI system and continuous-time quadratic cost
     * The discretization of the cost functions happens within this function. It employs an Euler-Discretization
     * @param linearSystem the discrete-time LTI system
     * @param costFunction the continuous-time cost function
     * @param stateOffset the offset for the affine system dynamics demanded by the LQOC Solver
     * @param dt the sampling time, required for discretization
     */
    void setFromTimeInvariantLinearQuadraticProblem(
        ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
        ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
        const ct::core::StateVector<STATE_DIM, SCALAR>& stateOffset,
        const double dt);

    //! return a flag indicating whether this LQOC Problem is constrained or not
    bool isConstrained() const;

    bool isInputBoxConstrained() const;
    bool isStateBoxConstrained() const;
    bool isGeneralConstrained() const;

    //! affine, time-varying system dynamics in discrete time
    ct::core::StateMatrixArray<STATE_DIM, SCALAR> A_;
    ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> B_;
    ct::core::StateVectorArray<STATE_DIM, SCALAR> b_;

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

    //! bounds of box constraints for input and state
    input_box_constr_array_t u_lb_;
    input_box_constr_array_t u_ub_;
    state_box_constr_array_t x_lb_;
    state_box_constr_array_t x_ub_;

    /*!
     * \brief container for the box constraint sparsity pattern
     * An example for how an element of this array might look like: [0 1 4 7]
     * This would mean that box constraints act on elements 0, 1, 4 and 7 of the state or input vector
     */
    input_box_constr_sparsity_array_t u_I_;
    state_box_constr_sparsity_array_t x_I_;

    //! the number of input box constraints at every stage.
    std::vector<int> nbu_;

    //! the number of state box constraints at every stage.
    std::vector<int> nbx_;

    //! general constraint lower bound
    constr_vec_array_t d_lb_;
    //! general constraint upper bound
    constr_vec_array_t d_ub_;
    //! linear general constraint matrices
    constr_state_jac_array_t C_;
    constr_control_jac_array_t D_;
    //! number of general inequality constraints
    std::vector<int> ng_;


private:
    //! the number of discrete time steps in the LOCP, not including terminal stage
    int K_;
};

}  // namespace optcon
}  // namespace ct
