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
 * The LQ problem hence has the following form:
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
 */
template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class LQOCProblem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    LQOCProblem(int N = 0);

    //! returns the number of discrete time steps in the LOCP, including terminal stage
    int getNumberOfStages();

    //! change the number of discrete time steps in the LOCP
    void changeNumStages(int N);

    //! set all member variables to zero
    void setZero();

    void setFromTimeInvariantLinearQuadraticProblem(ct::core::StateVector<STATE_DIM, SCALAR>& x0,
        ct::core::ControlVector<CONTROL_DIM, SCALAR>& u0,
        ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
        ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
        ct::core::StateVector<STATE_DIM, SCALAR>& stateOffset,
        double dt);

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

private:
    //! the number of discrete time steps in the LOCP, including terminal stage
    int K_;
};

}  // namespace optcon
}  // namespace ct
