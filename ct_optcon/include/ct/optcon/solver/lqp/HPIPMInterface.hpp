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

    typedef ct::core::StateMatrix<STATE_DIM> StateMatrix;
    typedef ct::core::StateMatrixArray<STATE_DIM> StateMatrixArray;
    typedef ct::core::ControlMatrix<CONTROL_DIM> ControlMatrix;
    typedef ct::core::ControlMatrixArray<CONTROL_DIM> ControlMatrixArray;
    typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM> StateControlMatrixArray;
    typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM> FeedbackArray;

    typedef ct::core::StateVectorArray<STATE_DIM> StateVectorArray;
    typedef ct::core::ControlVectorArray<CONTROL_DIM> ControlVectorArray;

    using box_constr_sparsity_t = typename LQOCProblem<STATE_DIM, CONTROL_DIM>::box_constr_sparsity_t;

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

    //! compute HPIPM's box constraint sparsity pattern based on ct LQOCProblem box constraint sparsity pattern
    /*!
     * todo document this method
     * @return number of box constraints
     */
    int get_hpipm_boxconstr_sp_pattern(const size_t n, const box_constr_sparsity_t& in,
        Eigen::Matrix<int, STATE_DIM + CONTROL_DIM, 1>& hpipm_sp_out);

    // temporary, get rid of that again todo
    void assembly_dynamic_constraint_container(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem,
        size_t nCon,
        size_t ind,
        const Eigen::Matrix<int, STATE_DIM + CONTROL_DIM, 1>& hpipm_sp);

    int N_;  //! horizon length

    std::vector<int> nx_;  //! number of states per stage
    std::vector<int> nu_;  //! number of inputs per stage

    std::vector<int> nb_;  //! number of box constraints per stage, currently always zero
    std::vector<int> ng_;  //! number of general constraints per stage, currently always zero

    std::vector<double*> hA_;                  //! system state sensitivities
    std::vector<double*> hB_;                  //! system input sensitivities
    std::vector<double*> hb_;                  //! system offset term
    StateVectorArray bEigen_;                  //! intermediate container for intuitive transcription
    Eigen::Matrix<double, state_dim, 1> hb0_;  //! intermediate container for intuitive transcription of first stage

    std::vector<double*> hQ_;                    //! pure state penalty hessian
    std::vector<double*> hS_;                    //! state-control cross-terms
    std::vector<double*> hR_;                    //! pure control penalty hessian
    std::vector<double*> hq_;                    //! pure state penalty jacobian
    std::vector<double*> hr_;                    //! pure control penalty jacobian
    Eigen::Matrix<double, control_dim, 1> hr0_;  //! intermediate container for intuitive transcription of first stage
    StateVectorArray hqEigen_;    //! interm. container for intuitive transcription of 1st order state penalty
    ControlVectorArray hrEigen_;  //! interm. container for intuitive transcription of 1st order control penalty

    std::vector<double*> hd_lb_;  //! lower box constraint threshold ?
    std::vector<double*> hd_ub_;  //! upper box constraint threshold ?
    std::vector<int*> hidxb_;     //! pointer sparsity pattern for box constraints
    ct::core::DiscreteArray<Eigen::Matrix<int, STATE_DIM + CONTROL_DIM, 1>>
        hdidxbEigen_;             //! container for box constraint sparsity pattern
    std::vector<double*> hd_lg_;  //! lower general constraint threshold ?
    std::vector<double*> hd_ug_;  //! upper general constraint threshold ?
    std::vector<double*> hC_;     //! general constraint jacobians w.r.t. states (presumably)
    std::vector<double*> hD_;     //! general constraint jacobians w.r.t. controls (presumably)

    double* x0_;  //! initial state

    // solution
    std::vector<double*> u_;       //! optimal control trajectory
    std::vector<double*> x_;       //! optimal state trajectory
    std::vector<double*> pi_;      //! todo what is this ?
    std::vector<double*> lam_lb_;  //! ptr to lagrange multiplier box-constraint lower
    std::vector<double*> lam_ub_;  //! ptr to lagrange multiplier box-constraint upper
    std::vector<double*> lam_lg_;  //! ptr to lagrange multiplier general-constraint lower
    std::vector<double*> lam_ug_;  //! ptr to lagrange multiplier general-constraint upper

    std::vector<Eigen::Matrix<double, -1, 1>> cont_lam_lb_;  //! container for lagr. mult. box-constraint lower
    std::vector<Eigen::Matrix<double, -1, 1>> cont_lam_ub_;  //! container for lagr. mult. box-constraint upper
    std::vector<Eigen::Matrix<double, -1, 1>> cont_lam_lg_;  //! container for lagr. mult. general-constraint lower
    std::vector<Eigen::Matrix<double, -1, 1>> cont_lam_ug_;  //! container for lagr. mult. general-constraint upper

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
