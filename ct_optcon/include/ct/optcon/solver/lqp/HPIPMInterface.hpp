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


private:
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
     * This method needs change coordinate systems, in the sense that
     *  \f[
     *  \mathbf x_{n+1} = \mathbf A_n \mathbf x_n + \mathbf B_n \mathbf u_n +\mathbf b_n
     *  + \hat \mathbf x_{n+1} - \mathbf A_n \hat \mathbf x_n -  \mathbf B_n \hat \mathbf u_n
     * \f]
     *
     * @todo potentially rename, as it only sets up the unconstrained part of the problem
     */
    void setupHPIPM(StateVectorArray& x,
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
     * @brief setup the constraint indices and matrices
     */
    void setupConstraints(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem);

    //! change number of states of the optimal control problem
    void changeNumberOfStages(int N);

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

    /*!
     * \brief  compute HPIPM's box constraint sparsity pattern based on ct LQOCProblem box constraint sparsity pattern
     * todo document this method
     * @return number of box constraints
     */
    int get_hpipm_boxconstr_sp_pattern(const size_t n,
        const box_constr_sparsity_t& in,
        box_constr_sparsity_t& hpipm_sp_out);

    /*!
     * \brief transcribe box constraints into the way required by HPIPM.
     * \note There are multiple reasons why the final, 'condensed' box constraint container
     * gets assembled here, rather than in the LQOC problem. Fristly, it allows the user to
     * independently specify state-box and control-box constraints. Secondly, this is relatively
     * specific to the implementation of HPIPM. Other LQ solvers might require different interfaces
     * for the box constraints.
     */
    void assemble_hpipm_box_constr_container(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem,
        size_t nCon,
        size_t ind,
        const box_constr_sparsity_t& hpipm_sp);

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
    //! lower box constraint boundary in hpipm spec
    constr_vec_array_t ux_lb_hpipm_;
    //! upper box constraint boundary in hpipm spec
    constr_vec_array_t ux_ub_hpipm_;
    /*!
     * \brief container for hpipm box constr. sparsity pattern
     * An example for how an element of this array might look like: [0 1 4 7]
     * This would mean that box constraints act on elements 0, 1, 4 and 7 of the
     * combined vector of decision variables [u; x]
     */
    ct::core::DiscreteArray<box_constr_sparsity_t> hdidxbEigen_;

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
    std::vector<Eigen::Matrix<double, max_box_constr_dim, 1>> cont_lam_lb_;
    //! container lagr. mult. box-constr. upper
    std::vector<Eigen::Matrix<double, max_box_constr_dim, 1>> cont_lam_ub_;
    //! container for lagr. mult. general-constraint lower
    std::vector<Eigen::Matrix<double, -1, 1>> cont_lam_lg_;
    //! container for lagr. mult. general-constraint upper
    std::vector<Eigen::Matrix<double, -1, 1>> cont_lam_ug_;

    ct::core::StateVectorArray<STATE_DIM> hx_;
    ct::core::StateVectorArray<STATE_DIM> hpi_;
    ct::core::ControlVectorArray<CONTROL_DIM> hu_;

    std::vector<char> qp_mem_;
    struct d_ocp_qp qp_;

    std::vector<char> qp_sol_mem_;
    struct d_ocp_qp_sol qp_sol_;

    struct d_ipm_hard_ocp_qp_arg arg_;  //! IPM settings
    std::vector<char> ipm_mem_;
    struct d_ipm_hard_ocp_qp_workspace workspace_;

    NLOptConSettings settings_;
};


}  // namespace optcon
}  // namespace ct

#endif  // HPIPM
