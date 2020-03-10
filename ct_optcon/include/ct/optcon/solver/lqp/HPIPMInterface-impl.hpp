/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef HPIPM

namespace ct {
namespace optcon {

template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::HPIPMInterface() : N_(-1), settings_(NLOptConSettings()), 
    dim_mem_(nullptr), qp_mem_(nullptr), qp_sol_mem_(nullptr), ipm_arg_mem_(nullptr), ipm_mem_(nullptr)
{
    hb0_.setZero();
    hr0_.setZero();

    configure(settings_);
}

template <int STATE_DIM, int CONTROL_DIM>
HPIPMInterface<STATE_DIM, CONTROL_DIM>::~HPIPMInterface()
{
    freeHpipmMemory();
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::initializeAndAllocate()
{
    freeHpipmMemory();

    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        std::cout << "HPIPM allocating memory for QP with time horizon: " << N_ << std::endl;
        for (int i = 0; i < N_ + 1; i++)
        {
            std::cout << "HPIPM stage " << i << ": (nx, nu, nbu, nbx, ng) : (" << nx_[i] << ", " << nu_[i] << ", "
                      << nbu_[i] << ", " << nbx_[i] << ", " << ng_[i] << ")" << std::endl;
        }
    }

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
        hidxbx_.data(), hlbx_.data(), hubx_.data(), hidxbu_.data(), hlbu_.data(), hubu_.data(),  // box constraints
        hC_.data(), hD_.data(), hlg_.data(), hug_.data(),                                        // gen constraints
        hZl_.data(), hZu_.data(), hzl_.data(), hzu_.data(), hidxs_.data(), hlls_.data(), hlus_.data(), &qp_);

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

    // create workspace
    int ipm_size = ::d_ocp_qp_ipm_ws_memsize(&dim_, &arg_);
    ipm_mem_ = malloc(ipm_size);
    ::d_ocp_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipm_mem_);


    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        std::cout << "HPIPM qp_size: " << qp_size << std::endl;
        std::cout << "HPIPM qp_sol_size: " << qp_sol_size << std::endl;
        std::cout << "HPIPM ipm_arg_size: " << ipm_arg_size << std::endl;
        std::cout << "HPIPM ipm_size: " << ipm_size << std::endl;
    }
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::freeHpipmMemory()
{
    free(dim_mem_);
    free(qp_mem_);
    free(qp_sol_mem_);
    free(ipm_arg_mem_);
    free(ipm_mem_);
    dim_mem_ = nullptr;
    qp_mem_ = nullptr;
    qp_sol_mem_ = nullptr;
    ipm_arg_mem_ = nullptr;
    ipm_mem_ = nullptr;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::configure(const NLOptConSettings& settings)
{
    settings_ = settings;
    ::d_ocp_qp_ipm_arg_set_iter_max(&settings_.lqoc_solver_settings.num_lqoc_iterations, &arg_);
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
        printf("\nhlg_\n");
        d_print_mat(1, ng_[i], hlg_[i], 1);
        printf("\nhug_\n");
        d_print_mat(1, ng_[i], hug_[i], 1);

    }   // end optional printout
#endif  // HPIPM_PRINT_MATRICES


    // solve optimal control problem
    ::d_ocp_qp_ipm_solve(&qp_, &qp_sol_, &arg_, &workspace_);
    ::d_ocp_qp_ipm_get_status(&workspace_, &hpipm_status_);

    isLrInvComputed_ = false;

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
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::computeStatesAndControls()
{
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

    // display iteration summary
    if (settings_.lqoc_solver_settings.lqoc_debug_print)
    {
        printSolution();
    }
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::computeLrInvArray()
{
    // extract data which is mandatory for computing either feedback or iLQR feedforward
    Eigen::Matrix<double, control_dim, control_dim> Lr;
    for (int i = 0; i < N_; i++)
    {
        ::d_ocp_qp_ipm_get_ric_Lr(i, &workspace_, Lr.data());
        Lr_inv_[i] = (Lr.template triangularView<Eigen::Lower>())
                         .solve(Eigen::Matrix<double, control_dim, control_dim>::Identity());
    }
    isLrInvComputed_ = true;
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::computeFeedbackMatrices()
{
    if (isLrInvComputed_ == false)
        computeLrInvArray();  // update Lr_inv_ first

    LQOCProblem<STATE_DIM, CONTROL_DIM>& p = *this->lqocProblem_;
    this->L_.resize(p.getNumberOfStages());

    // for steps k=1,...,N-1 we can just read Ls, Lr_inv already has been computed
    Eigen::Matrix<double, state_dim, control_dim> Ls;

    for (int i = 1; i < this->lqocProblem_->getNumberOfStages(); i++)
    {
        ::d_ocp_qp_ipm_get_ric_Ls(i, &workspace_, Ls.data());
        this->L_[i] = -(Ls * Lr_inv_[i]).transpose();
    }

    // for stage k=0, HPIPM does not have meaningful entries for Ls, so we have to manually design the feedback
    // retrieve Riccati matrix for stage 1 (we call it S, others call it P)
    Eigen::Matrix<double, state_dim, state_dim> S1;
    ::d_ocp_qp_ipm_get_ric_P(1, &workspace_, S1.data());

    // step2: compute G[0]
    Eigen::Matrix<double, control_dim, state_dim> G;
    G = p.P_[0];
    G.noalias() += p.B_[0].transpose() * S1 * p.A_[0];

    // step3: compute K[0] = H.inverse() * G
    this->L_[0] = (-Lr_inv_[0].transpose() * Lr_inv_[0] * G);
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::compute_lv()
{
    if (this->lqocProblem_->isConstrained())
        throw std::runtime_error(
            "Retrieving lv is not compatible with constraints in HPIPM. Switch to different algorithm, e.g. GNMS, "
            "which does not require lv");

    if (isLrInvComputed_ == false)
        computeLrInvArray();  // update Lr_inv_ first

    Eigen::Matrix<double, control_dim, 1> lr;
    for (int i = 0; i < this->lqocProblem_->getNumberOfStages(); i++)
    {
        ::d_ocp_qp_ipm_get_ric_lr(i, &workspace_, lr.data());
        this->lv_[i] = -Lr_inv_[i].transpose() * lr;
    }
}

template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::printSolution()
{
#ifdef HPIPM_PRINT_MATRICES
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
    bool dimsChanged = changeProblemSize(lqocProblem);

    // WARNING: the allocation should in practice not have to happen during the loop.
    // If possible, prefer receding horizon MPC problems.
    // If the number of stages has changed, however, the problem needs to be re-built:

    // setup unconstrained part of problem
    setupCostAndDynamics(lqocProblem->A_, lqocProblem->B_, lqocProblem->b_, lqocProblem->P_, lqocProblem->qv_,
        lqocProblem->Q_, lqocProblem->rv_, lqocProblem->R_);

    if (dimsChanged)
    {
        initializeAndAllocate();
    }
    else
    {
        // we need to call the setters to transform our data into HPIPM interal structures
        ::d_ocp_qp_set_all(hA_.data(), hB_.data(), hb_.data(), hQ_.data(), hS_.data(), hR_.data(), hq_.data(),
            hr_.data(), hidxbx_.data(), hlbx_.data(), hubx_.data(), hidxbu_.data(), hlbu_.data(), hubu_.data(),
            hC_.data(), hD_.data(), hlg_.data(), hug_.data(), hZl_.data(), hZu_.data(), hzl_.data(), hzu_.data(),
            hidxs_.data(), hlls_.data(), hlus_.data(), &qp_);
    }
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureInputBoxConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    bool configChanged = false;

    const int N = lqocProblem->getNumberOfStages();

    if ((int)nbu_.size() != N + 1)
    {
        nbu_.resize(N + 1, 0);
        hidxbu_.resize(N + 1);
        hlbu_.resize(N + 1);
        hubu_.resize(N + 1);
        configChanged = true;
    }

    // stages 1 to N
    for (int i = 0; i < N; i++)
    {
        if (nbu_[i] != lqocProblem->nbu_[i])
        {
            nbu_[i] = lqocProblem->nbu_[i];

            // set pointers to box constraint boundaries and sparsity pattern
            hlbu_[i] = lqocProblem->u_lb_[i].data();
            hubu_[i] = lqocProblem->u_ub_[i].data();
            hidxbu_[i] = lqocProblem->u_I_[i].data();

            configChanged = true;
        }
    }

    return configChanged;
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureStateBoxConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    bool configChanged = false;

    const int N = lqocProblem->getNumberOfStages();

    if ((int)nbx_.size() != N + 1)
    {
        nbx_.resize(N + 1, 0);
        hidxbx_.resize(N + 1);
        hlbx_.resize(N + 1);
        hubx_.resize(N + 1);
        configChanged = true;
    }

    // stages 1 to N + 1
    for (int i = 0; i < N + 1; i++)
    {
        // first stage requires special treatment as state is not a decision variable
        if (i == 0)
        {
            nbx_[i] = 0;
            continue;
        }

        if (nbx_[i] != lqocProblem->nbx_[i])
        {
            nbx_[i] = lqocProblem->nbx_[i];

            // set pointers to box constraint boundaries and sparsity pattern
            hlbx_[i] = lqocProblem->x_lb_[i].data();
            hubx_[i] = lqocProblem->x_ub_[i].data();
            hidxbx_[i] = lqocProblem->x_I_[i].data();

            configChanged = true;
        }
    }

    return configChanged;
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::configureGeneralConstraints(
    std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> lqocProblem)
{
    bool configChanged = false;

    const int N = lqocProblem->getNumberOfStages();
    if ((int)ng_.size() != N + 1)
    {
        ng_.resize(N + 1, 0);
        hlg_.resize(N + 1, 0);
        hug_.resize(N + 1, 0);
        hC_.resize(N + 1);
        hD_.resize(N + 1);
        configChanged = true;
    }


    // HPIPM-specific correction for first-stage general constraint bounds
    hd_lg_0_Eigen_ = lqocProblem->d_lb_[0];  // - lqocProblem->C_[0] * x0; // uncommented since x0=0
    hd_ug_0_Eigen_ = lqocProblem->d_ub_[0];  // - lqocProblem->C_[0] * x0; // uncommented since x0=0

    for (int i = 0; i < N + 1; i++)  // (includes the terminal stage)
    {
        // get the number of constraints
        if (ng_[i] != lqocProblem->ng_[i])
        {
            ng_[i] = lqocProblem->ng_[i];
            configChanged = true;
        }

        lqocProblem->C_[i].resize(lqocProblem->ng_[i], STATE_DIM);
        lqocProblem->D_[i].resize(lqocProblem->ng_[i], CONTROL_DIM);
        lqocProblem->d_lb_[i].resize(lqocProblem->ng_[i], 1);
        lqocProblem->d_ub_[i].resize(lqocProblem->ng_[i], 1);

        // set pointers to hpipm-style box constraint boundaries and sparsity pattern
        if (i == 0)
        {
            hlg_[i] = hd_lg_0_Eigen_.data();
            hug_[i] = hd_ug_0_Eigen_.data();
        }
        else
        {
            hlg_[i] = lqocProblem->d_lb_[i].data();
            hug_[i] = lqocProblem->d_ub_[i].data();
        }

        hC_[i] = lqocProblem->C_[i].data();
        hD_[i] = lqocProblem->D_[i].data();
    }

    return configChanged;
}


template <int STATE_DIM, int CONTROL_DIM>
void HPIPMInterface<STATE_DIM, CONTROL_DIM>::setupCostAndDynamics(StateMatrixArray& A,
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

    this->x_sol_[0].setZero();  // fixed intitial value problem (increment is zero)

    // IMPORTANT: for hb_ and hr_, we need a HPIPM-specific correction for the first stage
    hb0_ = b[0];   // + A[0] * x0; // uncommented since x0 = 0 in diff formulation
    hr0_ = rv[0];  // + P[0] * x0; // uncommented since x0 = 0 in diff formulation

    // assign data for LQ problem
    for (int i = 0; i < N_; i++)
    {
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
    hQ_[N_] = Q[N_].data();
    hq_[N_] = qv[N_].data();
}


template <int STATE_DIM, int CONTROL_DIM>
bool HPIPMInterface<STATE_DIM, CONTROL_DIM>::changeProblemSize(std::shared_ptr<LQOCProblem<STATE_DIM, CONTROL_DIM>> p)
{
    const int N = p->getNumberOfStages();

    // check if the problem horizon changed
    bool horizonChanged = false;
    if (N_ != N)
    {
        horizonChanged = true;
    }

    N_ = N;

    // check if the constraint configuration changed
    bool numConstraintsChanged = false;

    if (configureInputBoxConstraints(p))
        numConstraintsChanged = true;

    if (configureStateBoxConstraints(p))
        numConstraintsChanged = true;

    if (configureGeneralConstraints(p))
        numConstraintsChanged = true;


    // if neither the problem horizon nor the number of constraints changed, return false
    if (!horizonChanged && !numConstraintsChanged)
    {
        return false;
    }

    // resize control input variable counter
    if (nu_.size() > 0)
        nu_.back() = CONTROL_DIM;     // avoid error in case of increasing horizon
    nu_.resize(N_ + 1, CONTROL_DIM);  // initialize number of control inputs per stage
    nu_[N_] = 0;                      // last input is not a decision variable

    nx_.resize(N_ + 1, STATE_DIM);  // initialize number of states per stage
    nx_[0] = 0;                     // initial state is not a decision variable but given


    // resize the containers for the affine system dynamics approximation
    hA_.resize(N_);
    hB_.resize(N_);
    hb_.resize(N_);

    this->x_sol_.resize(N_ + 1);
    this->u_sol_.resize(N_);
    this->lv_.resize(N_);
    this->L_.resize(N_);

    // resize the containers for the LQ cost approximation
    hQ_.resize(N_ + 1);
    hS_.resize(N_ + 1);
    hR_.resize(N_ + 1);
    hq_.resize(N_ + 1);
    hr_.resize(N_ + 1);

    // currently ignored stuff (resized anyways)
    nsbx_.resize(N_ + 1, 0);  // no softened state box constraints
    nsbu_.resize(N_ + 1, 0);  // no softened input box constraints
    nsg_.resize(N_ + 1, 0);   // no softened general constraints

    hZl_.resize(N_ + 1, 0);
    hZu_.resize(N_ + 1, 0);
    hzl_.resize(N_ + 1, 0);
    hzu_.resize(N_ + 1, 0);

    hidxs_.resize(N_ + 1, 0);

    hlls_.resize(N_ + 1, 0);
    hlus_.resize(N_ + 1, 0);

    Lr_inv_.resize(N_);

    // assignments that stay constant despite varying problem impl data
    hb_[0] = hb0_.data();
    hr_[0] = hr0_.data();

    hS_[N_] = nullptr;
    hR_[N_] = nullptr;
    hr_[N_] = nullptr;

    return true;
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
const ct::core::ControlVectorArray<CONTROL_DIM>& HPIPMInterface<STATE_DIM, CONTROL_DIM>::get_lv()
{
    if (this->lqocProblem_->isConstrained())
        throw std::runtime_error(
            "Retrieving lv is not compatible with constraints in HPIPM. Switch to different algorithm, e.g. GNMS, "
            "which does not require lv");

    return this->lv_;
}

}  // namespace optcon
}  // namespace ct

#endif  // HPIPM
