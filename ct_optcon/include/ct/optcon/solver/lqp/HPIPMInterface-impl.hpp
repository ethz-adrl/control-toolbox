/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef HPIPM

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::HPIPMInterface(const int N, const int nb, const int ng)
    : N_(-1), nb_(1, nb), ng_(1, ng), x0_(nullptr), settings_(NLOptConSettings())
{
    // some zero variables
    hb0_.setZero();
    hr0_.setZero();

    // by default, set number of box and general constraints to zero
    if (N > 0)
        setSolverDimensions(N, nb, ng);

    configure(settings_);
}


template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::~HPIPMInterface()
{
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::setSolverDimensions(const int N, const int nb, const int ng)
{
    nb_.resize(N + 1, nb);
    ng_.resize(N + 1, ng);
    changeNumberOfStages(N);
    initializeAndAllocate();
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::initializeAndAllocate()
{
    int qp_size = ::d_memsize_ocp_qp(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data());
    qp_mem_.resize(qp_size);
    ::d_create_ocp_qp(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data(), &qp_, qp_mem_.data());

    int qp_sol_size = ::d_memsize_ocp_qp_sol(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data());
    qp_sol_mem_.resize(qp_sol_size);
    ::d_create_ocp_qp_sol(N_, nx_.data(), nu_.data(), nb_.data(), ng_.data(), &qp_sol_, qp_sol_mem_.data());

    int ipm_size = ::d_memsize_ipm_hard_ocp_qp(&qp_, &arg_);
    ipm_mem_.resize(ipm_size);
    ::d_create_ipm_hard_ocp_qp(&qp_, &arg_, &workspace_, ipm_mem_.data());

    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        std::cout << "HPIPM allocating memory for QP with time horizon: " << N_ << std::endl;
        for (size_t i = 0; i < N_ + 1; i++)
        {
            std::cout << "HPIPM stage " << i << ": (nx, nu, nb, ng) : (" << nx_[i] << ", " << nu_[i] << ", " << nb_[i]
                      << ", " << ng_[i] << ")" << std::endl;
        }
        std::cout << "HPIPM qp_size: " << qp_size << std::endl;
        std::cout << "HPIPM qp_sol_size: " << qp_sol_size << std::endl;
        std::cout << "HPIPM ipm_size: " << ipm_size << std::endl;
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configure(const NLOptConSettings& settings)
{
    settings_ = settings;

    arg_.iter_max = settings_.lqoc_solver_settings.num_lqoc_iterations;

    arg_.alpha_min = 1e-8;  // todo review and make setting
    arg_.mu_max = 1e-12;    // todo review and make setting
    arg_.mu0 = 2.0;         // todo review and make setting
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::solve()
{
// optional printout
#ifdef HPIPM_PRINT_MATRICES
    for (size_t i = 0; i < N_ + 1; i++)
    {
        std::cout << "HPIPM matrix printout for stage " << i << std::endl;
        if (i < N_)
        {
            printf("\nA\n");
            d_print_mat(STATE_DIM, STATE_DIM, hA_[i], STATE_DIM);
            printf("\nB\n");
            d_print_mat(STATE_DIM, CONTROL_DIM, hB_[i], STATE_DIM);
            printf("\nb\n");
            d_print_mat(1, STATE_DIM, hb_[i], 1);
        }

        printf("\nQ\n");
        d_print_mat(STATE_DIM, STATE_DIM, hQ_[i], STATE_DIM);
        printf("\nq\n");
        d_print_mat(1, STATE_DIM, hq_[i], 1);


        if (i < N_)
        {
            printf("\nR\n");
            d_print_mat(CONTROL_DIM, CONTROL_DIM, hR_[i], CONTROL_DIM);
            printf("\nS\n");
            d_print_mat(CONTROL_DIM, STATE_DIM, hS_[i], CONTROL_DIM);
            printf("\nr\n");
            d_print_mat(1, CONTROL_DIM, hr_[i], 1);
        }

        printf("\nnb\n");
        std::cout << nb_[i] << std::endl;
        printf("\nhidxb_\n");
        int_print_mat(1, nb_[i], hidxb_[i], 1);
        printf("\nhd_lb_\n");
        d_print_mat(1, nb_[i], hd_lb_[i], 1);
        printf("\nhd_ub_\n");
        d_print_mat(1, nb_[i], hd_ub_[i], 1);

        printf("\nng\n");
        std::cout << ng_[i] << std::endl;
        printf("\nC\n");
        d_print_mat(ng_[i], STATE_DIM, hC_[i], ng_[i]);
        printf("\nD\n");
        d_print_mat(ng_[i], CONTROL_DIM, hD_[i], ng_[i]);
        printf("\nhd_lg_\n");
        d_print_mat(1, ng_[i], hd_lg_[i], 1);
        printf("\nhd_ug_\n");
        d_print_mat(1, ng_[i], hd_ug_[i], 1);

    }   // end optional printout
#endif  // HPIPM_PRINT_MATRICES

    // set pointers to optimal control problem
    ::d_cvt_colmaj_to_ocp_qp(hA_.data(), hB_.data(), hb_.data(), hQ_.data(), hS_.data(), hR_.data(), hq_.data(),
        hr_.data(), hidxb_.data(), hd_lb_.data(), hd_ub_.data(), hC_.data(), hD_.data(), hd_lg_.data(), hd_ug_.data(),
        &qp_);

    // solve optimal control problem
    ::d_solve_ipm2_hard_ocp_qp(&qp_, &qp_sol_, &workspace_);

    // display iteration summary
    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        printf("\nipm iter = %d\n", workspace_.iter);
        printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
        d_print_e_tran_mat(5, workspace_.iter, workspace_.stat, 5);

        printSolution();
    }

    // extract state and control updates
    computeStateAndControlUpdates();
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::computeStateAndControlUpdates()
{
    LQOCProblem<STATE_DIM, CONTROL_DIM>& p = *this->lqocProblem_;

    // convert optimal control problem solution to standard column-major representation
    ::d_cvt_ocp_qp_sol_to_colmaj(&qp_, &qp_sol_, u_.data(), x_.data(), pi_.data(), lam_lb_.data(), lam_ub_.data(),
        lam_lg_.data(), lam_ug_.data());

    hx_[0] = this->lqocProblem_->x_[0];

    this->delta_x_norm_ = 0.0;
    this->delta_uff_norm_ = 0.0;

    this->lx_[0].setZero();

    for (int k = 0; k < this->lqocProblem_->getNumberOfStages(); k++)
    {
        // reconstruct control update
        this->lu_[k] = hu_[k] - p.u_[k];

        // reconstruct state update
        this->lx_[k + 1] = hx_[k + 1] - p.x_[k + 1];

        // compute the norms of the updates
        // TODO needed?
        this->delta_x_norm_ += this->lx_[k + 1].norm();
        this->delta_uff_norm_ += this->lu_[k].norm();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
ct::core::StateVectorArray<STATE_DIM> HPIPMInterface<STATE_DIM, CONTROL_DIM>::getSolutionState()
{
    return hx_;
}


template <int STATE_DIM, int CONTROL_DIM>
ct::core::ControlVectorArray<CONTROL_DIM> HPIPMInterface<STATE_DIM, CONTROL_DIM>::getSolutionControl()
{
    return hu_;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::getFeedback(ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>& K)
{
    LQOCProblem<STATE_DIM, CONTROL_DIM>& p = *this->lqocProblem_;
    K.resize(p.getNumberOfStages());

    // for stage 0, HPIPM does not provide feedback, so we have to construct it

    // step 1: reconstruct H[0]
    Eigen::Matrix<double, control_dim, control_dim> Lr;
    ::d_cvt_strmat2mat(Lr.rows(), Lr.cols(), &workspace_.L[0], 0, 0, Lr.data(), Lr.rows());
    Eigen::Matrix<double, control_dim, control_dim> H;
    H = Lr.template triangularView<Eigen::Lower>() * Lr.transpose();  // Lr is cholesky of H

    // step2: reconstruct S[1]
    Eigen::Matrix<double, state_dim, state_dim> Lq;
    ::d_cvt_strmat2mat(Lq.rows(), Lq.cols(), &workspace_.L[1], control_dim, control_dim, Lq.data(), Lq.rows());
    Eigen::Matrix<double, state_dim, state_dim> S;
    S = Lq.template triangularView<Eigen::Lower>() * Lq.transpose();  // Lq is cholesky of S

    // step3: compute G[0]
    Eigen::Matrix<double, control_dim, state_dim> G;
    G = p.P_[0];
    G.noalias() += p.B_[0].transpose() * S * p.A_[0];

    // step4: compute K[0]
    K[0] = (-H.inverse() * G);  // \todo use Lr here instead of H!


    // for all other steps we can just read Ls
    Eigen::Matrix<double, state_dim, control_dim> Ls;
    for (int i = 1; i < this->lqocProblem_->getNumberOfStages(); i++)
    {
        ::d_cvt_strmat2mat(Lr.rows(), Lr.cols(), &workspace_.L[i], 0, 0, Lr.data(), Lr.rows());
        ::d_cvt_strmat2mat(Ls.rows(), Ls.cols(), &workspace_.L[i], Lr.rows(), 0, Ls.data(), Ls.rows());
        K[i] = (-Ls * Lr.partialPivLu().inverse()).transpose();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
ct::core::ControlVectorArray<CONTROL_DIM> HPIPMInterface<STATE_DIM, CONTROL_DIM>::getFeedforwardUpdates()
{
    throw std::runtime_error("HPIPMInterface: getFeedforwardUpdates Not implemented");

    LQOCProblem<STATE_DIM, CONTROL_DIM>& p = *this->lqocProblem_;
    ct::core::ControlVectorArray<CONTROL_DIM> lv(p.getNumberOfStages());

    for (int i = 1; i < this->lqocProblem_->getNumberOfStages(); i++)
    {
        Eigen::Matrix<double, control_dim, control_dim> Lr;
        ::d_cvt_strmat2mat(Lr.rows(), Lr.cols(), &workspace_.L[i], 0, 0, Lr.data(), Lr.rows());

        Eigen::Matrix<double, 1, control_dim> llTranspose;
        ::d_cvt_strmat2mat(llTranspose.rows(), llTranspose.cols(), &workspace_.L[i], control_dim + state_dim, 0,
            llTranspose.data(), llTranspose.rows());

        lv[i] = -Lr.transpose().inverse() * llTranspose.transpose();
    }

    return lv;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::printSolution()
{
    int ii;

    ::d_cvt_ocp_qp_sol_to_colmaj(&qp_, &qp_sol_, u_.data(), x_.data(), pi_.data(), lam_lb_.data(), lam_ub_.data(),
        lam_lg_.data(), lam_ug_.data());

    printf("\nsolution\n\n");
    printf("\nu\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, nu_[ii], u_[ii], 1);
    printf("\nx\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, nx_[ii], x_[ii], 1);

#ifdef HPIPM_PRINT_MATRICES
    printf("\npi\n");
    for (ii = 0; ii < N_; ii++)
        d_print_mat(1, nx_[ii + 1], pi_[ii], 1);
    printf("\nlam_lb\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, nb_[ii], lam_lb_[ii], 1);
    printf("\nlam_ub\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, nb_[ii], lam_ub_[ii], 1);
    printf("\nlam_lg\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, ng_[ii], lam_lg_[ii], 1);
    printf("\nlam_ug\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, ng_[ii], lam_ug_[ii], 1);

    printf("\nt_lb\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, nb_[ii], (qp_sol_.t_lb + ii)->pa, 1);
    printf("\nt_ub\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, nb_[ii], (qp_sol_.t_ub + ii)->pa, 1);
    printf("\nt_lg\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, ng_[ii], (qp_sol_.t_lg + ii)->pa, 1);
    printf("\nt_ug\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, ng_[ii], (qp_sol_.t_ug + ii)->pa, 1);

    printf("\nresiduals\n\n");
    printf("\nres_g\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, nu_[ii] + nx_[ii], (workspace_.res_g + ii)->pa, 1);
    printf("\nres_b\n");
    for (ii = 0; ii < N_; ii++)
        d_print_e_mat(1, nx_[ii + 1], (workspace_.res_b + ii)->pa, 1);
    printf("\nres_m_lb\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, nb_[ii], (workspace_.res_m_lb + ii)->pa, 1);
    printf("\nres_m_ub\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, nb_[ii], (workspace_.res_m_ub + ii)->pa, 1);
    printf("\nres_m_lg\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, ng_[ii], (workspace_.res_m_lg + ii)->pa, 1);
    printf("\nres_m_ug\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, ng_[ii], (workspace_.res_m_ug + ii)->pa, 1);
    printf("\nres_d_lb\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, nb_[ii], (workspace_.res_d_lb + ii)->pa, 1);
    printf("\nres_d_ub\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, nb_[ii], (workspace_.res_d_ub + ii)->pa, 1);
    printf("\nres_d_lg\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, ng_[ii], (workspace_.res_d_lg + ii)->pa, 1);
    printf("\nres_d_ug\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_e_mat(1, ng_[ii], (workspace_.res_d_ug + ii)->pa, 1);
    printf("\nres_mu\n");
    printf("\n%e\n\n", workspace_.res_mu);
#endif  // HPIPM_PRINT_MATRICES

    printf("\nipm iter = %d\n", workspace_.iter);
    printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
    ::d_print_e_tran_mat(5, workspace_.iter, workspace_.stat, 5);
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::setProblemImpl(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // check if the number of stages N changed and adapt problem dimensions
    bool nStagesChanged = changeNumberOfStages(lqocProblem->getNumberOfStages());


    // WARNING: the allocation should in practice not have to happen during the loop.
    // If possible, prefer receding horizon MPC problems.
    // If the number of stages has changed, however, the problem needs to be re-built:
    if (nStagesChanged)
    {
        // update constraint configuration in case the horizon length has changed.
        if (lqocProblem->isBoxConstrained())
            configureBoxConstraints(lqocProblem);

        if (lqocProblem->isGeneralConstrained())
            configureGeneralConstraints(lqocProblem);
    }

    // setup unconstrained part of problem
    setupCostAndDynamics(lqocProblem->x_, lqocProblem->u_, lqocProblem->A_, lqocProblem->B_, lqocProblem->b_,
        lqocProblem->P_, lqocProblem->qv_, lqocProblem->Q_, lqocProblem->rv_, lqocProblem->R_);


    if (nStagesChanged)
    {
        initializeAndAllocate();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureBoxConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // stages 1 to N
    for (size_t i = 0; i < N_ + 1; i++)
    {
        nb_[i] = lqocProblem->nb_[i];

        // set pointers to box constraint boundaries and sparsity pattern
        hd_lb_[i] = lqocProblem->ux_lb_[i].data();
        hd_ub_[i] = lqocProblem->ux_ub_[i].data();
        hidxb_[i] = lqocProblem->ux_I_[i].data();

        // first stage requires special treatment as state is not a decision variable
        if (i == 0)
        {
            nb_[i] = 0;
            for (int j = 0; j < lqocProblem->nb_[i]; j++)
            {
                if (lqocProblem->ux_I_[i](j) < CONTROL_DIM)
                    nb_[i]++;  // adapt number of constraints such that only controls are listed as decision vars
                else
                    break;
            }
        }

        // TODO clarify with Gianluca if we need to reset the lagrange multiplier
        // before warmstarting (potentially wrong warmstart for the lambdas)

        // direct pointers of lagrange mult to corresponding containers
        lam_lb_[i] = cont_lam_lb_[i].data();
        lam_ub_[i] = cont_lam_ub_[i].data();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureGeneralConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // HPIPM-specific correction for first-stage general constraint bounds
    hd_lg_0_Eigen_ = lqocProblem->d_lb_[0] - lqocProblem->C_[0] * lqocProblem->x_[0];
    hd_ug_0_Eigen_ = lqocProblem->d_ub_[0] - lqocProblem->C_[0] * lqocProblem->x_[0];

    for (size_t i = 0; i < N_ + 1; i++)
    {
        // check dimensions
        assert(lqocProblem->d_lb_[i].rows() == lqocProblem->d_ub_[i].rows());
        assert(lqocProblem->d_lb_[i].rows() == lqocProblem->C_[i].rows());
        assert(lqocProblem->d_lb_[i].rows() == lqocProblem->D_[i].rows());
        assert(lqocProblem->C_[i].cols() == STATE_DIM);
        assert(lqocProblem->D_[i].cols() == CONTROL_DIM);

        // get the number of constraints
        ng_[i] = lqocProblem->ng_[i];

        // set pointers to hpipm-style box constraint boundaries and sparsity pattern
        if (i == 0)
        {
            hd_lg_[i] = hd_lg_0_Eigen_.data();
            hd_ug_[i] = hd_ug_0_Eigen_.data();
        }
        else
        {
            hd_lg_[i] = lqocProblem->d_lb_[i].data();
            hd_ug_[i] = lqocProblem->d_ub_[i].data();
        }
        hC_[i] = lqocProblem->C_[i].data();
        hD_[i] = lqocProblem->D_[i].data();

        // TODO clarify with Gianluca if we need to reset the lagrange multiplier
        // before warmstarting (potentially wrong warmstart for the lambdas)

        // direct pointers of lagrange mult to corresponding containers
        cont_lam_lg_[i].resize(ng_[i]);  // todo avoid dynamic allocation (e.g. by defining a max. constraint dim)
        cont_lam_ug_[i].resize(ng_[i]);  // todo avoid dynamic allocation (e.g. by defining a max. constraint dim)
        lam_lg_[i] = cont_lam_lg_[i].data();
        lam_ug_[i] = cont_lam_ug_[i].data();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::setupCostAndDynamics(StateVectorArray& x,
    ControlVectorArray& u,
    StateMatrixArray& A,
    StateControlMatrixArray& B,
    StateVectorArray& b,
    FeedbackArray& P,
    StateVectorArray& qv,
    StateMatrixArray& Q,
    ControlVectorArray& rv,
    ControlMatrixArray& R)
{
    if (N_ == -1)
        throw std::runtime_error("Time horizon not set, please set it first");

    // set the initial state
    x0_ = x[0].data();

    /*
     * transcribe the "differential" representation of the OptConProblem to the absolute origin of
     * the linear system.
     * Note: constant terms are not even handed over above (not important for solving LQ problem).
     */

    // STEP 1: transcription of affine system dynamics offset term
    for (int i = 0; i < N_; i++)
    {
        bEigen_[i] = b[i] + x[i + 1] - A[i] * x[i] - B[i] * u[i];
    }
    hb0_ = b[0] + x[1] - B[0] * u[0];  // this line needs to be transcribed separately (correction for first stage)


    // STEP 2: transcription of intermediate costs
    for (int i = 0; i < N_; i++)
    {
        hqEigen_[i] = qv[i] - Q[i] * x[i] - P[i].transpose() * u[i];
        hrEigen_[i] = rv[i] - R[i] * u[i] - P[i] * x[i];
    }
    hr0_ = hrEigen_[0] + P[0] * x[0];  // this line needs to be transcribed separately (correction for first stage)


    // STEP 3: transcription of terminal cost terms
    hqEigen_[N_] = qv[N_] - Q[N_] * x[N_];


    // STEP 4: The following quantities remain unchanged when changing coordinate systems
    for (int i = 0; i < N_; i++)
    {
        hA_[i] = A[i].data();
        hB_[i] = B[i].data();
    }

    // intermediate cost hessians and cross-terms
    for (int i = 0; i < N_; i++)
    {
        hQ_[i] = Q[i].data();
        hS_[i] = P[i].data();
        hR_[i] = R[i].data();
    }

    // final cost hessian state
    hQ_[N_] = Q[N_].data();

    // reset lqocProblem pointer, will get set in Base class if needed
    this->lqocProblem_ = nullptr;
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::changeNumberOfStages(int N)
{
    if (N_ == N)
        return false;  // return since problem is already correctly sized

    N_ = N;

    this->lx_.resize(N + 1);
    this->lu_.resize(N);

    nx_.resize(N_ + 1, STATE_DIM);    // initialize number of states per stage
    nu_.resize(N_ + 1, CONTROL_DIM);  // initialize number of control inputs per stage
    nb_.resize(N_ + 1, nb_.back());   // initialize number of box constraints per stage
    ng_.resize(N_ + 1, ng_.back());   // initialize number of general constraints per stage

    // resize the containers for the affine system dynamics approximation
    hA_.resize(N_);
    hB_.resize(N_);
    bEigen_.resize(N_);
    hb_.resize(N_);

    // resize the containers for the LQ cost approximation
    hQ_.resize(N_ + 1);
    hS_.resize(N_ + 1);
    hR_.resize(N_ + 1);
    hqEigen_.resize(N_ + 1);
    hq_.resize(N_ + 1);
    hrEigen_.resize(N_ + 1);
    hr_.resize(N_ + 1);

    hd_lb_.resize(N_ + 1);
    hd_ub_.resize(N_ + 1);
    hidxb_.resize(N_ + 1);
    hd_lg_.resize(N_ + 1);
    hd_ug_.resize(N_ + 1);
    hC_.resize(N_ + 1);
    hD_.resize(N_ + 1);

    u_.resize(N_ + 1);
    x_.resize(N_ + 1);
    pi_.resize(N_);
    hx_.resize(N_ + 1);
    hpi_.resize(N_);
    hu_.resize(N_);

    lam_lb_.resize(N_ + 1);
    lam_ub_.resize(N_ + 1);
    lam_lg_.resize(N_ + 1);
    lam_ug_.resize(N_ + 1);
    cont_lam_lb_.resize(N_ + 1);
    cont_lam_ub_.resize(N_ + 1);
    cont_lam_lg_.resize(N_ + 1);
    cont_lam_ug_.resize(N_ + 1);

    for (int i = 0; i < N_; i++)
    {
        // first state and last input are not optimized
        x_[i + 1] = hx_[i + 1].data();
        u_[i] = hu_[i].data();
    }
    for (int i = 0; i < N_; i++)
    {
        pi_[i] = hpi_[i].data();
    }
    for (int i = 0; i < N_; i++)
    {
        hq_[i] = hqEigen_[i].data();
        hr_[i] = hrEigen_[i].data();
        hb_[i] = bEigen_[i].data();
    }

    hS_[N_] = nullptr;
    hR_[N_] = nullptr;

    hq_[N_] = hqEigen_[N_].data();
    hr_[N_] = nullptr;

    hb_[0] = hb0_.data();
    hr_[0] = hr0_.data();

    ct::core::StateVectorArray<STATE_DIM> hx;
    ct::core::ControlVectorArray<CONTROL_DIM> hu;

    // initial state is not a decision variable but given
    nx_[0] = 0;

    // last input is not a decision variable
    nu_[N] = 0;

    return true;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::d_zeros(double** pA, int row, int col)
{
    *pA = (double*)malloc((row * col) * sizeof(double));
    double* A = *pA;
    int i;
    for (i = 0; i < row * col; i++)
        A[i] = 0.0;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::d_print_mat(int m, int n, double* A, int lda)
{
    int i, j;
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            printf("%9.5f ", A[i + lda * j]);
        }
        printf("\n");
    }
    printf("\n");
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::d_print_tran_mat(int row, int col, double* A, int lda)
{
    int i, j;
    for (j = 0; j < col; j++)
    {
        for (i = 0; i < row; i++)
        {
            printf("%9.5f ", A[i + lda * j]);
        }
        printf("\n");
    }
    printf("\n");
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::d_print_e_mat(int m, int n, double* A, int lda)
{
    int i, j;
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            printf("%e\t", A[i + lda * j]);
        }
        printf("\n");
    }
    printf("\n");
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::d_print_e_tran_mat(int row, int col, double* A, int lda)
{
    int i, j;
    for (j = 0; j < col; j++)
    {
        for (i = 0; i < row; i++)
        {
            printf("%e\t", A[i + lda * j]);
        }
        printf("\n");
    }
    printf("\n");
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::int_print_mat(int row, int col, int* A, int lda)
{
    int i, j;
    for (i = 0; i < row; i++)
    {
        for (j = 0; j < col; j++)
        {
            printf("%d ", A[i + lda * j]);
        }
        printf("\n");
    }
    printf("\n");
}

}  // namespace optcon
}  // namespace ct

#endif  // HPIPM
