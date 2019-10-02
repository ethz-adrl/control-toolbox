/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define HPIPM  // todo remove
//#define HPIPM_PRINT_MATRICES  // todo remove

#ifdef HPIPM

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::HPIPMInterface(const int N, const int nb, const int ng)
    : N_(-1), nb_(1, nb), ng_(1, ng), x0_(nullptr), settings_(NLOptConSettings())
{
    std::cout << "entering constructor " << std::endl;  // todo remove
    // some zero variables
    hb0_.setZero();
    hr0_.setZero();

    // by default, set number of box and general constraints to zero
    if (N > 0)
        setSolverDimensions(N, nb, ng);

    configure(settings_);
    std::cout << "leaving constructor " << std::endl;  // todo remove
}


template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::~HPIPMInterface()
{
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::setSolverDimensions(const int N, const int nb, const int ng)
{
    std::cout << "entering set solver dim " << std::endl;  // todo remove

    nb_.resize(N + 1, nb);  // todo deprecate
    nbx_.resize(N + 1, 0);  // todo replace 0 by correct value
    nbu_.resize(N + 1, 0);  // tdo replace 0 by correct value
    ng_.resize(N + 1, ng);

    nsbx_.resize(N + 1, 0);  // todo correct val
    nsbu_.resize(N + 1, 0);  // todo correct val
    nsg_.resize(N + 1, 0);   // todo correct val

    changeNumberOfStages(N);
    initializeAndAllocate();

    std::cout << "leaving set solver dim " << std::endl;  // todo remove
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::initializeAndAllocate()
{
    std::cout << "entering init and allocate " << std::endl;  // todo remove

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
    //  ::d_ocp_qp_ipm_arg_set_mu0(&mu0_, &arg_); // todo bring back those options
    //  ::d_ocp_qp_ipm_arg_set_iter_max(&iter_max_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_alpha_min(&alpha_min_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_mu0(&mu0_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_warm_start(&warm_start_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr_, &arg_);
    //  ::d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg_, &arg_);

    // create workspace
    int ipm_size = ::d_ocp_qp_ipm_ws_memsize(&dim_, &arg_);
    ipm_mem_ = malloc(ipm_size);
    ::d_ocp_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipm_mem_);


    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        std::cout << "HPIPM allocating memory for QP with time horizon: " << N_ << std::endl;
        for (int i = 0; i < N_ + 1; i++)
        {
            std::cout << "HPIPM stage " << i << ": (nx, nu, nb, ng) : (" << nx_[i] << ", " << nu_[i] << ", " << nb_[i]
                      << ", " << ng_[i] << ")" << std::endl;
        }
        std::cout << "HPIPM qp_size: " << qp_size << std::endl;
        std::cout << "HPIPM qp_sol_size: " << qp_sol_size << std::endl;
        std::cout << "HPIPM ipm_arg_size: " << ipm_arg_size << std::endl;
        std::cout << "HPIPM ipm_size: " << ipm_size << std::endl;
    }

    std::cout << "leaving init and allocate" << std::endl;  // todo remove
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configure(const NLOptConSettings& settings)
{
    std::cout << "entering configure " << std::endl;  // todo remove
    settings_ = settings;

    /*
    arg_.iter_max = settings_.lqoc_solver_settings.num_lqoc_iterations;

    arg_.alpha_min = 1e-8;  // todo review and make setting
    arg_.mu0 = 2.0;         // todo review and make setting
                            // arg_.mu_max = 1e-12;    // todo review and make setting
*/
    std::cout << "leaving configure " << std::endl;  // todo remove
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

        printf("\nnb\n");
        std::cout << nb_[i] << std::endl;
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

        printSolution();
    }

    // convert optimal control problem solution to standard column-major representation
    //    ::d_cvt_ocp_qp_sol_to_colmaj(&qp_, &qp_sol_, u_.data(), x_.data(), pi_.data(), lam_lb_.data(), lam_ub_.data(),
    //        lam_lg_.data(), lam_ug_.data());

    // extract solution for x and u
    int ii;
    for (ii = 0; ii < N_; ii++)
    {
        Eigen::Matrix<double, CONTROL_DIM, 1> u_sol;
        ::d_ocp_qp_sol_get_u(ii, &qp_sol_, this->u_sol_[ii].data());
    }
    for (ii = 0; ii <= N_; ii++)
    {
        Eigen::Matrix<double, STATE_DIM, 1> x_sol;
        ::d_ocp_qp_sol_get_x(ii, &qp_sol_, this->x_sol_[ii].data());
    }

    // and design the feedback matrices
    designFeedback();
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
    std::cout << "H matrix " << std::endl << H << std::endl;

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
    for (ii = 0; ii <= N_; ii++)
    {
        Eigen::Matrix<double, CONTROL_DIM, 1> u_sol;
        ::d_ocp_qp_sol_get_u(ii, &qp_sol_, u_sol.data());
        std::cout << u_sol << std::endl;
    }

    std::cout << "Solution for x: " << std::endl;
    for (ii = 0; ii <= N_; ii++)
    {
        Eigen::Matrix<double, STATE_DIM, 1> x_sol;
        ::d_ocp_qp_sol_get_x(ii, &qp_sol_, x_sol.data());
        std::cout << x_sol.transpose() << std::endl;
    }

    // todo test all of the following
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

/*
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
*/
/*
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
    */
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
    std::cout << "entering set problem impl " << std::endl;

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


    //   if (nStagesChanged)
    //  {
    initializeAndAllocate();
    //  }

    std::cout << "leaving set problem impl " << std::endl;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureBoxConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    // stages 1 to N
    for (int i = 0; i < N_ + 1; i++)
    {
        nb_[i] = lqocProblem->nb_[i];

        // set pointers to box constraint boundaries and sparsity pattern
        hd_lb_[i] = lqocProblem->ux_lb_[i].data();
        hd_ub_[i] = lqocProblem->ux_ub_[i].data();  // todo replace this by below action

        // hlbx_[i] =   //! todo assign pointer
        // hubx_[i] =   //! todo assign pointer
        // hlbu_[i] =   //! todo assign pointer
        // hubu_[i] =   //! todo assign pointer

        // hidxb_[i] = lqocProblem->ux_I_[i].data(); // todo deprecate

        // hidxbx_[i] =  //! todo assign pointer
        // hidxbu_[i] =  //! todo assign pointer

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

    // assign data for LQ problem

    // set pointer to the initial state
    this->x_sol_[0] = x[0];
    //x_[0] = x[0].data();
    x0_ = x[0].data();

    for (int i = 0; i < N_; i++)
    {
        x_[i] = x[i].data();
        u_[i] = u[i].data();
        hA_[i] = A[i].data();
        hB_[i] = B[i].data();
        hb_[i] = b[i].data();
    }

    for (int i = 0; i < N_; i++)
    {
        hQ_[i] = Q[i].data();
        hS_[i] = P[i].data();
        hR_[i] = R[i].data();
        hq_[i] = qv[i].data();
        hr_[i] = rv[i].data();
    }

    // terminal stage
    x_[N_] = x[N_].data();
    hQ_[N_] = Q[N_].data();
    hq_[N_] = qv[N_].data();

    // IMPORTANT: for hb_ and hr_, we need a HPIPM-specific correction for the first stage
    hb0_ = b[0] + A[0] * x[0];
    hr0_ = rv[0] + P[0] * x[0];
    hb_[0] = hb0_.data();
    hr_[0] = hr0_.data();

    // reset lqocProblem pointer, will get set in Base class if needed
    this->lqocProblem_ = nullptr;
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::changeNumberOfStages(int N)
{
    if (N_ == N)
        return false;  // return since problem is already correctly sized

    N_ = N;

    nx_.resize(N_ + 1, STATE_DIM);    // initialize number of states per stage
    nu_.resize(N_ + 1, CONTROL_DIM);  // initialize number of control inputs per stage
    nb_.resize(N_ + 1, nb_.back());   // initialize number of box constraints per stage
    ng_.resize(N_ + 1, ng_.back());   // initialize number of general constraints per stage

    nbu_.resize(N + 1, 0);   // todo correct val
    nbx_.resize(N + 1, 0);   // todo correct val
    nsbx_.resize(N + 1, 0);  // todo correct val
    nsbu_.resize(N + 1, 0);  // todo correct val
    nsg_.resize(N + 1, 0);   // todo correct val

    hd_lb_.resize(N + 1, 0);  // todo correct val
    hd_ub_.resize(N + 1, 0);  // todo correct val
    hlbx_.resize(N + 1, 0);   // todo correct val
    hubx_.resize(N + 1, 0);   // todo correct val
    hlbu_.resize(N + 1, 0);   // todo correct val
    hubu_.resize(N + 1, 0);   // todo correct val

    hlg_.resize(N + 1, 0);  // todo correct val
    hug_.resize(N + 1, 0);  // todo correct val
    hZl_.resize(N + 1, 0);  // todo correct val
    hZu_.resize(N + 1, 0);  // todo correct val
    hzl_.resize(N + 1, 0);  // todo correct val
    hzu_.resize(N + 1, 0);  // todo correct val

    hidxs_.resize(N + 1, 0);  // todo correct val

    hlls_.resize(N + 1, 0);  // todo correct val
    hlus_.resize(N + 1, 0);  // todo correct val

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

    hd_lb_.resize(N_ + 1);
    hd_ub_.resize(N_ + 1);
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
        x_[i + 1] = this->x_sol_[i + 1].data();
        u_[i] = this->u_sol_[i].data();
    }
    for (int i = 0; i < N_; i++)
    {
        pi_[i] = hpi_[i].data();
    }

    hS_[N_] = nullptr;
    hR_[N_] = nullptr;
    hr_[N_] = nullptr;

    hb_[0] = nullptr;
    hr_[0] = nullptr;

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
