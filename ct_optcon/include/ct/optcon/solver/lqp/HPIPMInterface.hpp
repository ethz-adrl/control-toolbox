/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "LQOCSolver.hpp"

#ifdef HPIPM

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_aux_ext_dep.h>

extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_timing.h>
}

#include <unsupported/Eigen/MatrixFunctions>


namespace ct {
namespace optcon {

/*!
 * This class implements an interface to the HPIPM solver
 * 
 * \note see also the official hpipm repo, i.e. the discussion in https://github.com/giaf/hpipm/issues/76
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
    HPIPMInterface();

    //! destructor
    virtual ~HPIPMInterface();

    virtual void configure(const NLOptConSettings& settings) override;

    void solve() override;

    virtual void computeStatesAndControls() override;
    virtual void computeFeedbackMatrices() override;
    virtual void compute_lv() override;

    void printSolution();

    //! brief setup and configure the box constraints
    virtual bool configureInputBoxConstraints(
        std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;
    virtual bool configureStateBoxConstraints(
        std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

    //! brief setup and configure the general (in)equality constraints
    virtual bool configureGeneralConstraints(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

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

    //! override this method to catch corner case with lv being incompatible with constraints
    virtual const ct::core::ControlVectorArray<CONTROL_DIM>& get_lv() override;

private:
    void setSolverDimensions(const int N, const int nbu = 0, const int nbx = 0, const int ng = 0);

    /*!
     * @brief set problem implementation for HPIPM
     * \warning This method is called in the control loop. As little memory as possible
     * should be allocated in this function. Ideally this method only sets pointers.
     *
     * \warning If you wish to
     */
    void setProblemImpl(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem) override;

    /*!
     * @brief transcribe the problem for HPIPM
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
     *
     *
     * \warning To achieve compatibility with HPIPM, this method needs to perform a change of coordinates for certain problem variables in the first stage.
     */
    void setupCostAndDynamics(StateMatrixArray& A,
        StateControlMatrixArray& B,
        StateVectorArray& b,
        FeedbackArray& P,
        StateVectorArray& qv,
        StateMatrixArray& Q,
        ControlVectorArray& rv,
        ControlMatrixArray& R);

    /*!
     * @brief change the size of the optimal control problem
     * @return true if the problem size changed, otherwise return false
     */
    bool changeProblemSize(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem);

    /**
     * @brief compute the array of inverses of the Lr-matrix from the cholesky factorization of the Hessian
     * @note is put into separate function since triggered by request and flagged by bool
     */
    void computeLrInvArray();
    bool isLrInvComputed_;

    //! prints a matrix in column-major format
    void d_print_mat(int m, int n, double* A, int lda);

    //! prints the transposed of a matrix in column-major format, exp notation
    void d_print_e_tran_mat(int row, int col, double* A, int lda);

    //! frees memory allocated for the HPIPM data structures
    void freeHpipmMemory();

    //! horizon length
    int N_;

    std::vector<int> nx_;  //! number of states per stage
    std::vector<int> nu_;  //! number of inputs per stage

    std::vector<int> nbu_;  //! number of input box constraints per stage
    std::vector<int> nbx_;  //! number of state box constraints per stage

    std::vector<int> ng_;  //! number of general constraints per stage

    std::vector<int> nsbx_;  // number of softed constraints on state box constraints
    std::vector<int> nsbu_;  // number of softed constraints on input box constraints
    std::vector<int> nsg_;   // number of softed constraints on general constraints

    std::vector<double*> hA_;                  //! system state sensitivities
    std::vector<double*> hB_;                  //! system input sensitivities
    std::vector<double*> hb_;                  //! system offset term
    Eigen::Matrix<double, state_dim, 1> hb0_;  //! intermediate container for intuitive transcription of first stage

    std::vector<double*> hQ_;                    //! pure state penalty hessian
    std::vector<double*> hS_;                    //! state-control cross-terms
    std::vector<double*> hR_;                    //! pure control penalty hessian
    std::vector<double*> hq_;                    //! pure state penalty jacobian
    std::vector<double*> hr_;                    //! pure control penalty jacobian
    Eigen::Matrix<double, control_dim, 1> hr0_;  //! local Eigen container for first stage transcription

    std::vector<double*> hlbx_;  //! pointer to lower state box constraint boundary
    std::vector<double*> hubx_;  //! pointer to upper state box constraint boundary
    std::vector<double*> hlbu_;  //! pointer to lower input box constraint boundary
    std::vector<double*> hubu_;  //! pointer to upper input box constraint boundary

    std::vector<int*> hidxbx_;  //! pointer to sparsity pattern for box constraints in x
    std::vector<int*> hidxbu_;  //! pointer to sparsity pattern for box constraints in u

    std::vector<double*> hlg_;  //! lower general constraint boundary
    std::vector<double*> hug_;  //! upper general constraint boundary
    std::vector<double*> hC_;   //! general constraint jacobians w.r.t. states
    std::vector<double*> hD_;   //! general constraint jacobians w.r.t. controls

    //  local vars for constraint bounds for statge k=0, which need to be different by HPIPM convention
    Eigen::VectorXd hd_lg_0_Eigen_;
    Eigen::VectorXd hd_ug_0_Eigen_;

    std::vector<double*> hZl_;  // todo what are those quantities? (related to soft constraints?)
    std::vector<double*> hZu_;
    std::vector<double*> hzl_;
    std::vector<double*> hzu_;
    std::vector<int*> hidxs_;
    std::vector<double*> hlls_;
    std::vector<double*> hlus_;

    // cached data for efficiency
    ct::core::DiscreteArray<ct::core::ControlMatrix<control_dim>> Lr_inv_;  // inv of cholesky matrix of hessian H


    //! settings from NLOptConSolver
    NLOptConSettings settings_;

    //! ocp qp dimensions
    int dim_size_;
    void* dim_mem_;
    struct d_ocp_qp_dim dim_;

    void* qp_mem_;
    struct d_ocp_qp qp_;

    void* qp_sol_mem_;
    struct d_ocp_qp_sol qp_sol_;

    void* ipm_arg_mem_;
    struct d_ocp_qp_ipm_arg arg_;

    // workspace
    void* ipm_mem_;
    struct d_ocp_qp_ipm_ws workspace_;
    int hpipm_status_;  // status code after solving

    // todo make this a setting
    ::hpipm_mode mode_ = ::hpipm_mode::SPEED;  // ROBUST/BALANCED; see also hpipm_common.h
};

}  // namespace optcon
}  // namespace ct

#endif  // HPIPM
