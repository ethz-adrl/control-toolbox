/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef HPIPM

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::HPIPMInterface(const int N, const int nbu, const int nbx, const int ng)
    : N_(-1), nbu_(1, nbu), nbx_(1, nbx), ng_(1, ng), settings_(NLOptConSettings())
{
    hb0_.setZero();
    hr0_.setZero();

    // by default, set number of box and general constraints to zero
    if (N > 0)
        setSolverDimensions(N, nbu, nbx, ng);

    configure(settings_);
}


template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::~HPIPMInterface()
{
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::setSolverDimensions(const int N,
    const int nbu,
    const int nbx,
    const int ng)
{
    nbu_.resize(N + 1, nbu);  // todo this is bad design, is there no other way to propagate the number of constraints?
    nbx_.resize(N + 1, nbx);  // todo this is bad design, is there no other way to propagate the number of constraints?
    ng_.resize(N + 1, ng);

    changeNumberOfStages(N);
    initializeAndAllocate();
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::initializeAndAllocate()
{
    // allocate ocp dimensions
    dim_size_ = d_ocp_qp_dim_memsize(N_);
    dim_mem_ = malloc(dim_size_);
    d_ocp_qp_dim_create(N_, &dim_, dim_mem_);
    d_ocp_qp_dim_set_all(
        nx_.data(), nu_.data(), nbx_.data(), nbu_.data(), ng_.data(), nsbx_.data(), nsbu_.data(), nsg_.data(), &dim_);

    // allocate ocp qp
    int qp_size = ::d_ocp_qp_memsize(&dim_);
    qp_mem_ = malloc(qp_size);
    ::d_ocp_qp_create(&dim_, &qp_, qp_mem_);
    ::d_ocp_qp_set_all(hA_.data(), hB_.data(), hb_.data(), hQ_.data(), hS_.data(), hR_.data(), hq_.data(), hr_.data(),
        hidxbx_.data(), hlbx_.data(), hubx_.data(), hidxbu_.data(), hlbu_.data(), hubu_.data(), hC_.data(), hD_.data(),
        hlg_.data(), hug_.data(), hZl_.data(), hZu_.data(), hzl_.data(), hzu_.data(), hidxs_.data(), hlls_.data(),
        hlus_.data(), &qp_);

    // allocation for solution
    int qp_sol_size = ::d_ocp_qp_sol_memsize(&dim_);
    qp_sol_mem_ = malloc(qp_sol_size);
    ::d_ocp_qp_sol_create(&dim_, &qp_sol_, qp_sol_mem_);

    // ipm arg
    int ipm_arg_size = ::d_ocp_qp_ipm_arg_memsize(&dim_);
    ipm_arg_mem_ = malloc(ipm_arg_size);
    ::d_ocp_qp_ipm_arg_create(&dim_, &arg_, ipm_arg_mem_);

    ::d_ocp_qp_ipm_arg_set_default(mode_, &arg_);
    ::d_ocp_qp_ipm_arg_set_iter_max(&settings_.lqoc_solver_settings.num_lqoc_iterations, &arg_);
    ::d_ocp_qp_ipm_arg_set_mu0(&settings_.lqoc_solver_settings.mu0, &arg_);
    ::d_ocp_qp_ipm_arg_set_alpha_min(&settings_.lqoc_solver_settings.alpha_min, &arg_);
    ::d_ocp_qp_ipm_arg_set_mu0(&settings_.lqoc_solver_settings.mu0, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_stat(&settings_.lqoc_solver_settings.tol_stat, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_eq(&settings_.lqoc_solver_settings.tol_eq, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_ineq(&settings_.lqoc_solver_settings.tol_ineq, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_comp(&settings_.lqoc_solver_settings.tol_comp, &arg_);
    ::d_ocp_qp_ipm_arg_set_reg_prim(&settings_.lqoc_solver_settings.reg_prim, &arg_);
    ::d_ocp_qp_ipm_arg_set_warm_start(&settings_.lqoc_solver_settings.warm_start, &arg_);
    ::d_ocp_qp_ipm_arg_set_pred_corr(&settings_.lqoc_solver_settings.pred_corr, &arg_);
    ::d_ocp_qp_ipm_arg_set_ric_alg(&settings_.lqoc_solver_settings.ric_alg, &arg_);

    // create workspace
    int ipm_size = ::d_ocp_qp_ipm_ws_memsize(&dim_, &arg_);
    ipm_mem_ = malloc(ipm_size);
    ::d_ocp_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipm_mem_);


    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        std::cout << "HPIPM allocating memory for QP with time horizon: " << N_ << std::endl;
        for (int i = 0; i < N_ + 1; i++)
        {
            std::cout << "HPIPM stage " << i << ": (nx, nu, nbu, nbx, ng) : (" << nx_[i] << ", " << nu_[i] << ", "
                      << nbu_[i] << ", " << nbx_[i] << ", " << ng_[i] << ")" << std::endl;
        }
        std::cout << "HPIPM qp_size: " << qp_size << std::endl;
        std::cout << "HPIPM qp_sol_size: " << qp_sol_size << std::endl;
        std::cout << "HPIPM ipm_arg_size: " << ipm_arg_size << std::endl;
        std::cout << "HPIPM ipm_size: " << ipm_size << std::endl;
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configure(const NLOptConSettings& settings)
{
    settings_ = settings;
    ::d_ocp_qp_ipm_arg_set_iter_max(&settings_.lqoc_solver_settings.num_lqoc_iterations, &arg_);
    ::d_ocp_qp_ipm_arg_set_mu0(&settings_.lqoc_solver_settings.mu0, &arg_);
    ::d_ocp_qp_ipm_arg_set_alpha_min(&settings_.lqoc_solver_settings.alpha_min, &arg_);
    ::d_ocp_qp_ipm_arg_set_mu0(&settings_.lqoc_solver_settings.mu0, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_stat(&settings_.lqoc_solver_settings.tol_stat, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_eq(&settings_.lqoc_solver_settings.tol_eq, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_ineq(&settings_.lqoc_solver_settings.tol_ineq, &arg_);
    ::d_ocp_qp_ipm_arg_set_tol_comp(&settings_.lqoc_solver_settings.tol_comp, &arg_);
    ::d_ocp_qp_ipm_arg_set_reg_prim(&settings_.lqoc_solver_settings.reg_prim, &arg_);
    ::d_ocp_qp_ipm_arg_set_warm_start(&settings_.lqoc_solver_settings.warm_start, &arg_);
    ::d_ocp_qp_ipm_arg_set_pred_corr(&settings_.lqoc_solver_settings.pred_corr, &arg_);
    ::d_ocp_qp_ipm_arg_set_ric_alg(&settings_.lqoc_solver_settings.ric_alg, &arg_);
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::solve()
{
// optional printout
#ifdef HPIPM_PRINT_MATRICES
    for (int i = 0; i < N_ + 1; i++)
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
            printf("\nu\n");
            d_print_mat(1, CONTROL_DIM, u_[i], 1);
        }

        if (i > 0)
        {
            printf("\nx\n");
            d_print_mat(1, STATE_DIM, x_[i], 1);
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

        printf("\nnbu\n");
        std::cout << nbu_[i] << std::endl;
        printf("\nhlbu_\n");
        d_print_mat(1, nbu_[i], hlbu_[i], 1);
        printf("\nhubu_\n");
        d_print_mat(1, nbu_[i], hubu_[i], 1);

        printf("\nnbx\n");
        std::cout << nbx_[i] << std::endl;
        printf("\nhlbx_\n");
        d_print_mat(1, nbx_[i], hlbx_[i], 1);
        printf("\nhubx_\n");
        d_print_mat(1, nbx_[i], hubx_[i], 1);

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


    // solve optimal control problem
    ::d_ocp_qp_ipm_solve(&qp_, &qp_sol_, &arg_, &workspace_);
    ::d_ocp_qp_ipm_get_status(&workspace_, &hpipm_status_);

    // display iteration summary
    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        printf("\nHPIPM returned with flag %i.\n", hpipm_status_);
        if (hpipm_status_ == 0)
        {
            printf("\n -> QP solved!\n");
        }
        else if (hpipm_status_ == 1)
        {
            printf("\n -> Solver failed! Maximum number of iterations reached\n");
        }
        else if (hpipm_status_ == 2)
        {
            printf("\n -> Solver failed! Minimum step length reached\n");
        }
        else if (hpipm_status_ == 2)
        {
            printf("\n -> Solver failed! NaN in computations\n");
        }
        else
        {
            printf("\n -> Solver failed! Unknown return flag\n");
        }
        printf("\nipm iter = %d\n", workspace_.iter);
        printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
        d_print_e_tran_mat(5, workspace_.iter, workspace_.stat, 5);
    }

    // extract solution for x and u
    for (int ii = 0; ii < N_; ii++)
    {
        Eigen::Matrix<double, CONTROL_DIM, 1> u_sol;
        ::d_ocp_qp_sol_get_u(ii, &qp_sol_, this->u_sol_[ii].data());
    }
    for (int ii = 0; ii <= N_; ii++)
    {
        Eigen::Matrix<double, STATE_DIM, 1> x_sol;
        ::d_ocp_qp_sol_get_x(ii, &qp_sol_, this->x_sol_[ii].data());
    }

    // and design the feedback matrices
    designFeedback();

    // display iteration summary
    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        printSolution();
    }
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::designFeedback()
{
    LQOCProblem<STATE_DIM, CONTROL_DIM>& p = *this->lqocProblem_;
    this->L_.resize(p.getNumberOfStages(), Eigen::Matrix<double, control_dim, state_dim>::Zero());

    // for stage 0, HPIPM does not provide feedback, so we have to construct it
    // step 1: reconstruct H[0]
    Eigen::Matrix<double, control_dim, control_dim> Lr;
    ::blasfeo_unpack_dmat(Lr.rows(), Lr.cols(), &workspace_.L[0], 0, 0, Lr.data(), Lr.rows());
    Eigen::Matrix<double, control_dim, control_dim> H;
    H = Lr.template triangularView<Eigen::Lower>() * Lr.transpose();  // Lr is cholesky of H

    // step2: reconstruct S[1]
    Eigen::Matrix<double, state_dim, state_dim> Lq;
    ::blasfeo_unpack_dmat(Lq.rows(), Lq.cols(), &workspace_.L[1], control_dim, control_dim, Lq.data(), Lq.rows());
    Eigen::Matrix<double, state_dim, state_dim> S;
    S = Lq.template triangularView<Eigen::Lower>() * Lq.transpose();  // Lq is cholesky of S

    // step3: compute G[0]
    Eigen::Matrix<double, control_dim, state_dim> G;
    G = p.P_[0];
    G.noalias() += p.B_[0].transpose() * S * p.A_[0];

    // step4: compute K[0]
    this->L_[0] = (-H.inverse() * G);  // \todo use Lr here instead of H!

    // for all other steps we can just read Ls
    Eigen::Matrix<double, state_dim, control_dim> Ls;
    for (int i = 1; i < this->lqocProblem_->getNumberOfStages(); i++)
    {
        Eigen::Map<Eigen::Matrix<double, control_dim, control_dim>> Lr_map(workspace_.L[i].pA);

        ::blasfeo_unpack_dmat(Lr.rows(), Lr.cols(), &workspace_.L[i], 0, 0, Lr.data(), Lr.rows());
        ::blasfeo_unpack_dmat(Ls.rows(), Ls.cols(), &workspace_.L[i], Lr.rows(), 0, Ls.data(), Ls.rows());
        this->L_[i] = (-Ls * Lr_map.partialPivLu().inverse()).transpose();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::printSolution()
{
    int ii;

    std::cout << "Solution for u: " << std::endl;
    for (ii = 0; ii < N_; ii++)
    {
        std::cout << this->u_sol_[ii].transpose() << std::endl;
    }

    std::cout << "Solution for x: " << std::endl;
    for (ii = 0; ii <= N_; ii++)
    {
        std::cout << this->x_sol_[ii].transpose() << std::endl;
    }

#ifdef HPIPM_PRINT_MATRICES
    printf("\npi\n");
    for (ii = 0; ii < N_; ii++)
        d_print_mat(1, nx_[ii + 1], pi_[ii], 1);
    printf("\nlam_lb\n");
    printf("\nlam_lg\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, ng_[ii], lam_lg_[ii], 1);
    printf("\nlam_ug\n");
    for (ii = 0; ii <= N_; ii++)
        d_print_mat(1, ng_[ii], lam_ug_[ii], 1);

#endif  // HPIPM_PRINT_MATRICES

    int iter;
    ::d_ocp_qp_ipm_get_iter(&workspace_, &iter);
    printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
    ::d_print_mat(5, iter, workspace_.stat, 5);
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
        if (lqocProblem->isInputBoxConstrained())
            configureInputBoxConstraints(lqocProblem);

        if (lqocProblem->isStateBoxConstrained())
            configureStateBoxConstraints(lqocProblem);

        if (lqocProblem->isGeneralConstrained())
            configureGeneralConstraints(lqocProblem);
    }

    // setup unconstrained part of problem
    setupCostAndDynamics(lqocProblem->x_, lqocProblem->u_, lqocProblem->A_, lqocProblem->B_, lqocProblem->b_,
        lqocProblem->P_, lqocProblem->qv_, lqocProblem->Q_, lqocProblem->rv_, lqocProblem->R_);

    if (nStagesChanged)
        initializeAndAllocate();
    else
    {
        // TODO: it is not exactly clear why we need to do this here ... but otherwise the solver fails!
        ::d_ocp_qp_set_all(hA_.data(), hB_.data(), hb_.data(), hQ_.data(), hS_.data(), hR_.data(), hq_.data(),
            hr_.data(), hidxbx_.data(), hlbx_.data(), hubx_.data(), hidxbu_.data(), hlbu_.data(), hubu_.data(),
            hC_.data(), hD_.data(), hlg_.data(), hug_.data(), hZl_.data(), hZu_.data(), hzl_.data(), hzu_.data(),
            hidxs_.data(), hlls_.data(), hlus_.data(), &qp_);
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureInputBoxConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // stages 1 to N
    for (int i = 0; i < N_; i++)
    {
        nbu_[i] = lqocProblem->nbu_[i];

        // set pointers to box constraint boundaries and sparsity pattern
        hlbu_[i] = lqocProblem->u_lb_[i].data();
        hubu_[i] = lqocProblem->u_ub_[i].data();
        hidxbu_[i] = lqocProblem->u_I_[i].data();
    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureStateBoxConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // stages 1 to N
    for (int i = 0; i < N_ + 1; i++)
    {
        nbx_[i] = lqocProblem->nbx_[i];

        // set pointers to box constraint boundaries and sparsity pattern
        hlbx_[i] = lqocProblem->x_lb_[i].data();
        hubx_[i] = lqocProblem->x_ub_[i].data();
        hidxbx_[i] = lqocProblem->x_I_[i].data();

        // first stage requires special treatment as state is not a decision variable
        if (i == 0)
        {
            nbx_[i] = 0;
        }

    }
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureGeneralConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // HPIPM-specific correction for first-stage general constraint bounds
    hd_lg_0_Eigen_ = lqocProblem->d_lb_[0] - lqocProblem->C_[0] * lqocProblem->x_[0];
    hd_ug_0_Eigen_ = lqocProblem->d_ub_[0] - lqocProblem->C_[0] * lqocProblem->x_[0];

    for (int i = 0; i < N_ + 1; i++)  // (includes the terminal stage)
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

    // transcribe initial state into solution vector (won't be considered by HPIPM)
    this->x_sol_[0] = x[0];

    // IMPORTANT: for hb_ and hr_, we need a HPIPM-specific correction for the first stage
    hb0_ = b[0] + A[0] * x[0];
    hr0_ = rv[0] + P[0] * x[0];

    // assign data for LQ problem
    for (int i = 0; i < N_; i++)
    {
        x_[i] = x[i].data();
        u_[i] = u[i].data();
        hA_[i] = A[i].data();
        hB_[i] = B[i].data();

        if (i > 0)
            hb_[i] = b[i].data();  // do not mutate pointers for init stage
    }

    for (int i = 0; i < N_; i++)
    {
        hQ_[i] = Q[i].data();
        hS_[i] = P[i].data();
        hR_[i] = R[i].data();
        hq_[i] = qv[i].data();

        if (i > 0)
            hr_[i] = rv[i].data();  // do not mutate pointers for init stage
    }

    // terminal stage
    x_[N_] = x[N_].data();
    hQ_[N_] = Q[N_].data();
    hq_[N_] = qv[N_].data();
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::changeNumberOfStages(int N)
{
    if (N_ == N)
        return false;  // return since problem is already correctly sized

    N_ = N;

    // resize control input variable counter
    if (nu_.size() > 0)
        nu_.back() = CONTROL_DIM;     // avoid error in case of increasing horizon
    nu_.resize(N_ + 1, CONTROL_DIM);  // initialize number of control inputs per stage
    nu_[N_] = 0;                      // last input is not a decision variable

    nx_.resize(N_ + 1, STATE_DIM);  // initialize number of states per stage
    nx_[0] = 0;                     // initial state is not a decision variable but given

    nbu_.resize(N_ + 1, nbu_.back());  // initialize number of box constraints per stage
    nbx_.resize(N_ + 1, nbx_.back());  // initialize number of box constraints per stage
    ng_.resize(N_ + 1, ng_.back());    // initialize number of general constraints per stage

    nbu_.resize(N_ + 1, 0);   // todo correct val
    nbx_.resize(N_ + 1, 0);   // todo correct val
    nsbx_.resize(N_ + 1, 0);  // todo correct val
    nsbu_.resize(N_ + 1, 0);  // todo correct val
    nsg_.resize(N_ + 1, 0);   // todo correct val

    hlbx_.resize(N_ + 1, 0);   // todo correct val
    hubx_.resize(N_ + 1, 0);   // todo correct val
    hlbu_.resize(N_ + 1, 0);   // todo correct val
    hubu_.resize(N_ + 1, 0);   // todo correct val

    hlg_.resize(N_ + 1, 0);  // todo correct val
    hug_.resize(N_ + 1, 0);  // todo correct val
    hZl_.resize(N_ + 1, 0);  // todo correct val
    hZu_.resize(N_ + 1, 0);  // todo correct val
    hzl_.resize(N_ + 1, 0);  // todo correct val
    hzu_.resize(N_ + 1, 0);  // todo correct val

    hidxs_.resize(N_ + 1, 0);  // todo correct val

    hlls_.resize(N_ + 1, 0);  // todo correct val
    hlus_.resize(N_ + 1, 0);  // todo correct val

    // resize the containers for the affine system dynamics approximation
    hA_.resize(N_);
    hB_.resize(N_);
    hb_.resize(N_);

    // resize the containers for the LQ cost approximation
    hQ_.resize(N_ + 1);
    hS_.resize(N_ + 1);
    hR_.resize(N_ + 1);
    hq_.resize(N_ + 1);
    hr_.resize(N_ + 1);

    hidxbx_.resize(N_ + 1);
    hidxbu_.resize(N_ + 1);
    hd_lg_.resize(N_ + 1);
    hd_ug_.resize(N_ + 1);
    hC_.resize(N_ + 1);
    hD_.resize(N_ + 1);

    u_.resize(N_ + 1);
    x_.resize(N_ + 1);
    pi_.resize(N_);
    hpi_.resize(N_);
    this->x_sol_.resize(N_ + 1);
    this->u_sol_.resize(N_);

    lam_lg_.resize(N_ + 1);
    lam_ug_.resize(N_ + 1);
    cont_lam_lg_.resize(N_ + 1);
    cont_lam_ug_.resize(N_ + 1);

    // assignments that stay constant despite varying problem impl data
    for (int i = 0; i < N_; i++)
    {
        pi_[i] = hpi_[i].data();
    }

    hb_[0] = hb0_.data();
    hr_[0] = hr0_.data();

    hS_[N_] = nullptr;
    hR_[N_] = nullptr;
    hr_[N_] = nullptr;

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
