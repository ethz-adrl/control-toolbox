/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "LQOCSolver.hpp"

#ifdef HPIPM

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_ipm_hard.h>
}

#include <unsupported/Eigen/MatrixFunctions>


namespace ct {
namespace optcon {

/*!
 * This class implements an interface to the HPIPM solver
 *
 * \warning in order to allow for an efficient implementation of constrained MPC,
 * the configuration of the box and general constraints must be done independently
 * from setProblem()
 */
template <int STATE_DIM, int CONTROL_DIM>
class HPIPMInterface : public LQOCSolver<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int state_dim = STATE_DIM;
    static const int control_dim = CONTROL_DIM;
    static const int max_box_constr_dim = STATE_DIM + CONTROL_DIM;

    using StateMatrix = ct::core::StateMatrix<STATE_DIM>;
    using StateMatrixArray = ct::core::StateMatrixArray<STATE_DIM>;
    using ControlMatrix = ct::core::ControlMatrix<CONTROL_DIM>;
    using ControlMatrixArray = ct::core::ControlMatrixArray<CONTROL_DIM>;
    using StateControlMatrixArray = ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM>;
    using FeedbackArray = ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>;

    using StateVectorArray = ct::core::StateVectorArray<STATE_DIM>;
    using ControlVectorArray = ct::core::ControlVectorArray<CONTROL_DIM>;

    // definitions for variable-size constraints
    using constr_vec_t = Eigen::Matrix<double, -1, 1>;
    using constr_vec_array_t = ct::core::DiscreteArray<constr_vec_t>;


    //! typedef a container for a sparsity pattern vector for box constraints
    using box_constr_sparsity_t = Eigen::Matrix<int, max_box_constr_dim, 1>;

    //! constructor
    HPIPMInterface(int N = -1);

    //! destructor
    virtual ~HPIPMInterface();

    virtual void configure(const NLOptConSettings& settings) override;

    void solve() override;

    virtual void computeStateAndControlUpdates() override;

    virtual ct::core::StateVectorArray<STATE_DIM> getSolutionState() override;

    virtual ct::core::ControlVectorArray<CONTROL_DIM> getSolutionControl() override;

    virtual void getFeedback(ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>& K) override;

    virtual ct::core::ControlVectorArray<CONTROL_DIM> getFeedforwardUpdates() override;

    void printSolution();

    //! brief setup and configure the box constraints
    virtual void configureBoxConstraints(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

    //! brief setup and configure the general (in)equality constraints
    virtual void configureGeneralConstraints(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

    /*!
     * @brief allocate memory for HPIPM
     *
     * \warning These functions are assumed to be used only outside the control loop.
     * Their intended use is to call them before the loop to setup the memory needed by
     * the QP structures and solver (dynamic memory allocation is time consuming).
     *
     * Needs to be called when
     *  - the number of stages changes
     *  - the box constraint configuration changes
     *  - the general constraint configuration changes
     */
    virtual void initializeAndAllocate() override;

private:
    /*!
     * @brief set problem implementation for hpipm
     * \warning This method is called in the loop. As little memory as possible
     * should be allocated in this function. Ideally this method only sets pointers.
     *
     * \warning If you wish to
     */
    void setProblemImpl(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

    /*!
     * @brief transcribe the problem from original local formulation to HPIPM's global coordinates
     *
     * See also the description of the LQOC Problem in class LQOCProblem.h
     *
     * @param x current state reference \f$ \hat \mathbf x_n \f$
     * @param u current control reference  \f$ \hat \mathbf u_n \f$
     * @param A affine system matrices \f$ \mathbf A_n \f$
     * @param B affine system matrices \f$ \mathbf B_n \f$
     * @param b affine system matrix \f$ \mathbf b_n \f$
     * @param P mixed cost term \f$ \mathbf P_n \f$
     * @param qv pure state-cost term \f$ \mathbf q_n \f$ (first order derivative)
     * @param Q pure state-cost term \f$ \mathbf Q_n \f$ (second order derivative)
     * @param rv pure input-cost term \f$ \mathbf r_n \f$ (first order derivative)
     * @param R pure input-cost term \f$ \mathbf R_n \f$ (second order derivative)
     * @param keepPointers keep pointers
     *
     *
     * This method needs change coordinate systems, in the sense that
     *  \f[
     *  \mathbf x_{n+1} = \mathbf A_n \mathbf x_n + \mathbf B_n \mathbf u_n +\mathbf b_n
     *  + \hat \mathbf x_{n+1} - \mathbf A_n \hat \mathbf x_n -  \mathbf B_n \hat \mathbf u_n
     * \f]
     */
    void setupCostAndDynamics(StateVectorArray& x,
        ControlVectorArray& u,
        StateMatrixArray& A,
        StateControlMatrixArray& B,
        StateVectorArray& b,
        FeedbackArray& P,
        StateVectorArray& qv,
        StateMatrixArray& Q,
        ControlVectorArray& rv,
        ControlMatrixArray& R,
        bool keepPointers = false);

    /*!
     * @brief change number of states of the optimal control problem
     * @return true if number of stages changed, false if number of stages is unchanged.
     */
    bool changeNumberOfStages(int N);

    //! creates a zero matrix
    void d_zeros(double** pA, int row, int col);

    //! prints a matrix in column-major format
    void d_print_mat(int m, int n, double* A, int lda);

    //! prints a matrix in column-major format (exponential notation)
    void d_print_e_mat(int m, int n, double* A, int lda);

    //! prints the transposed of a matrix in column-major format (exponential notation)
    void d_print_e_tran_mat(int row, int col, double* A, int lda);

    //! prints the transposed of a matrix in column-major format
    void d_print_tran_mat(int row, int col, double* A, int lda);

    //! prints a matrix in column-major format
    void int_print_mat(int row, int col, int* A, int lda);

    //! horizon length
    int N_;

    //! number of states per stage
    std::vector<int> nx_;
    //! number of inputs per stage
    std::vector<int> nu_;

    //! number of box constraints per stage
    std::vector<int> nb_;
    //! number of general constraints per stage
    std::vector<int> ng_;

    //! initial state
    double* x0_;

    //! system state sensitivities
    std::vector<double*> hA_;
    //! system input sensitivities
    std::vector<double*> hB_;
    //! system offset term
    std::vector<double*> hb_;
    //! intermediate container for intuitive transcription of system representation from local to global coordinates
    StateVectorArray bEigen_;
    //! intermediate container for intuitive transcription of first stage from local to global coordinates
    Eigen::Matrix<double, state_dim, 1> hb0_;


    //! pure state penalty hessian
    std::vector<double*> hQ_;
    //! state-control cross-terms
    std::vector<double*> hS_;
    //! pure control penalty hessian
    std::vector<double*> hR_;
    //! pure state penalty jacobian
    std::vector<double*> hq_;
    //! pure control penalty jacobian
    std::vector<double*> hr_;
    //! intermediate container for intuitive transcription of first stage from local to global coordinates
    Eigen::Matrix<double, control_dim, 1> hr0_;
    //! interm. container for intuitive transcription of 1st order state penalty from local to global coordinates
    StateVectorArray hqEigen_;
    //! interm. container for intuitive transcription of 1st order control penalty from local to global coordinates
    ControlVectorArray hrEigen_;


    //! pointer to lower box constraint boundary
    std::vector<double*> hd_lb_;
    //! pointer to upper box constraint boundary
    std::vector<double*> hd_ub_;
    //! pointer to sparsity pattern for box constraints
    std::vector<int*> hidxb_;

    //! lower general constraint boundary
    std::vector<double*> hd_lg_;
    //! upper general constraint boundary
    std::vector<double*> hd_ug_;
    //! general constraint jacobians w.r.t. states
    std::vector<double*> hC_;
    //! general constraint jacobians w.r.t. controls (presumably)
    std::vector<double*> hD_;


    /*
     * SOLUTION variables
     */
    //! optimal control trajectory
    std::vector<double*> u_;
    //! optimal state trajectory
    std::vector<double*> x_;
    //! @todo what is this ?
    std::vector<double*> pi_;
    //! ptr to lagrange multiplier box-constraint lower
    std::vector<double*> lam_lb_;
    //! ptr to lagrange multiplier box-constraint upper
    std::vector<double*> lam_ub_;
    //! ptr to lagrange multiplier general-constraint lower
    std::vector<double*> lam_lg_;
    //! ptr to lagrange multiplier general-constraint upper
    std::vector<double*> lam_ug_;

    //! container lagr. mult. box-constr. lower
    ct::core::DiscreteArray<Eigen::Matrix<double, max_box_constr_dim, 1>> cont_lam_lb_;
    //! container lagr. mult. box-constr. upper
    ct::core::DiscreteArray<Eigen::Matrix<double, max_box_constr_dim, 1>> cont_lam_ub_;
    //! container for lagr. mult. general-constraint lower
    ct::core::DiscreteArray<Eigen::Matrix<double, -1, 1>> cont_lam_lg_;
    //! container for lagr. mult. general-constraint upper
    ct::core::DiscreteArray<Eigen::Matrix<double, -1, 1>> cont_lam_ug_;

    ct::core::StateVectorArray<STATE_DIM> hx_;
    ct::core::StateVectorArray<STATE_DIM> hpi_;
    ct::core::ControlVectorArray<CONTROL_DIM> hu_;

    //! settings from NLOptConSolver
    NLOptConSettings settings_;

    std::vector<char> qp_mem_;
    struct d_ocp_qp qp_;

    std::vector<char> qp_sol_mem_;
    struct d_ocp_qp_sol qp_sol_;

    struct d_ipm_hard_ocp_qp_arg arg_;  //! IPM settings
    std::vector<char> ipm_mem_;
    struct d_ipm_hard_ocp_qp_workspace workspace_;
};


}  // namespace optcon
}  // namespace ct

#endif  // HPIPM
