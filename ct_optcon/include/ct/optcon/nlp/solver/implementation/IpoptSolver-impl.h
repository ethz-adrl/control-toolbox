/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/nlp/solver/IpoptSolver.h>

// #define DEBUG_PRINT


template <typename SCALAR>
IpoptSolver<SCALAR>::IpoptSolver(std::shared_ptr<tpl::Nlp<SCALAR>> nlp, const NlpSolverSettings& settings)
    : BASE(nlp, settings), settings_(BASE::settings_.ipoptSettings_)
{
    //Constructor arguments
    //Argument 1: create console output
    //Argument 2: create empty
    ipoptApp_ = std::shared_ptr<Ipopt::IpoptApplication>(new Ipopt::IpoptApplication(true, false));
    configureDerived(settings);
}

template <typename SCALAR>
IpoptSolver<SCALAR>::~IpoptSolver()
{
    // Needed to destruct all the IPOPT memory
    Ipopt::Referencer* t = NULL;
    this->ReleaseRef(t);
    delete t;
}

template <typename SCALAR>
void IpoptSolver<SCALAR>::configureDerived(const NlpSolverSettings& settings)
{
    std::cout << "calling Ipopt configure derived" << std::endl;
    settings_ = settings.ipoptSettings_;
    setSolverOptions();
    this->isInitialized_ = true;
}

template <typename SCALAR>
void IpoptSolver<SCALAR>::setSolverOptions()
{
    ipoptApp_->Options()->SetNumericValue("tol", settings_.tol_);
    ipoptApp_->Options()->SetNumericValue("constr_viol_tol", settings_.constr_viol_tol_);
    ipoptApp_->Options()->SetIntegerValue("max_iter", settings_.max_iter_);
    // ipoptApp_->Options()->SetNumericValue("resto.tol", settings_.restoTol_);
    // ipoptApp_->Options()->SetNumericValue("acceptable_tol", settings_.acceptableTol_);
    // ipoptApp_->Options()->SetNumericValue("resto.acceptable_tol", settings_.restoAcceptableTol_);
    ipoptApp_->Options()->SetStringValueIfUnset("linear_scaling_on_demand", settings_.linear_scaling_on_demand_);
    ipoptApp_->Options()->SetStringValueIfUnset("hessian_approximation", settings_.hessian_approximation_);
    // ipoptApp_->Options()->SetStringValueIfUnset("nlp_scaling_method", settings_.nlp_scaling_method_);
    ipoptApp_->Options()->SetIntegerValue("print_level", settings_.printLevel_);  //working now
    ipoptApp_->Options()->SetStringValueIfUnset("print_user_options", settings_.print_user_options_);
    // ipoptApp_->Options()->SetIntegerValue("print_frequency_iter", settings_.print_frequency_iter_);
    ipoptApp_->Options()->SetStringValueIfUnset("derivative_test", settings_.derivativeTest_);
    ipoptApp_->Options()->SetIntegerValue("print_level", settings_.printLevel_);
    ipoptApp_->Options()->SetNumericValue("derivative_test_tol", settings_.derivativeTestTol_);
    ipoptApp_->Options()->SetNumericValue("derivative_test_perturbation", settings_.derivativeTestPerturbation_);
    ipoptApp_->Options()->SetNumericValue("point_perturbation_radius", settings_.point_perturbation_radius_);
    ipoptApp_->Options()->SetStringValueIfUnset("linear_system_scaling", settings_.linearSystemScaling_);
    ipoptApp_->Options()->SetStringValueIfUnset("linear_solver", settings_.linear_solver_);
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::solve()
{
    status_ = ipoptApp_->Initialize();
    if (!(status_ == Ipopt::Solve_Succeeded) && !this->isInitialized_)
        throw(std::runtime_error("NLP initialization failed"));

    // Ask Ipopt to solve the problem
    status_ = ipoptApp_->OptimizeTNLP(this);

    if (status_ == Ipopt::Solve_Succeeded || status_ == Ipopt::Solved_To_Acceptable_Level)
    {
        // Retrieve some statistics about the solve
        if (settings_.printLevel_ > 1)
        {
            Ipopt::Index iter_count = ipoptApp_->Statistics()->IterationCount();
            std::cout << std::endl
                      << std::endl
                      << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

            SCALAR final_obj = ipoptApp_->Statistics()->FinalObjective();
            std::cout << std::endl
                      << std::endl
                      << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
        }
        return true;
    }
    else
    {
        if (settings_.printLevel_ > 1)
            std::cout << " ipopt return value: " << status_ << std::endl;
        return false;
    }
}

template <typename SCALAR>
void IpoptSolver<SCALAR>::prepareWarmStart(size_t maxIterations)
{
    ipoptApp_->Options()->SetStringValue("warm_start_init_point", "yes");
    ipoptApp_->Options()->SetNumericValue("warm_start_bound_push", 1e-9);
    ipoptApp_->Options()->SetNumericValue("warm_start_bound_frac", 1e-9);
    ipoptApp_->Options()->SetNumericValue("warm_start_slack_bound_frac", 1e-9);
    ipoptApp_->Options()->SetNumericValue("warm_start_slack_bound_push", 1e-9);
    ipoptApp_->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-9);
    ipoptApp_->Options()->SetIntegerValue("max_iter", (int)maxIterations);
    ipoptApp_->Options()->SetStringValue("derivative_test", "none");
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::get_nlp_info(Ipopt::Index& n,
    Ipopt::Index& m,
    Ipopt::Index& nnz_jac_g,
    Ipopt::Index& nnz_h_lag,
    IndexStyleEnum& index_style)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering get_nlp_info()" << std::endl;
#endif
    n = this->nlp_->getVarCount();
    assert(n == n);

    m = this->nlp_->getConstraintsCount();
    assert(m == m);

    nnz_jac_g = static_cast<Ipopt::Index>(this->nlp_->getNonZeroJacobianCount());
    assert(nnz_jac_g == nnz_jac_g);

    if (settings_.hessian_approximation_ == "exact")
        nnz_h_lag = static_cast<Ipopt::Index>(this->nlp_->getNonZeroHessianCount());

    index_style = Ipopt::TNLP::C_STYLE;

#ifdef DEBUG_PRINT
    std::cout << "... number of decision variables = " << n << std::endl;
    std::cout << "... number of constraints = " << m << std::endl;
    std::cout << "... nonzeros in jacobian = " << nnz_jac_g << std::endl;
#endif

    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::get_bounds_info(Ipopt::Index n,
    SCALAR* x_l,
    SCALAR* x_u,
    Ipopt::Index m,
    SCALAR* g_l,
    SCALAR* g_u)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering get_bounds_info()" << std::endl;
#endif  //DEBUG_PRINT
    MapVecXs x_lVec(x_l, n);
    MapVecXs x_uVec(x_u, n);
    MapVecXs g_lVec(g_l, m);
    MapVecXs g_uVec(g_u, m);
    this->nlp_->getVariableBounds(x_lVec, x_uVec, n);
    // bounds on optimization vector
    // x_l <= x <= x_u
    assert(x_l == x_l);
    assert(x_u == x_u);
    assert(n == n);

    // constraints bounds (e.g. for equality constraints = 0)
    this->nlp_->getConstraintBounds(g_lVec, g_uVec, m);
    assert(g_l == g_l);
    assert(g_u == g_u);

#ifdef DEBUG_PRINT
    std::cout << "... Leaving get_bounds_info()" << std::endl;
#endif  //DEBUG_PRINT

    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::get_starting_point(Ipopt::Index n,
    bool init_x,
    SCALAR* x,
    bool init_z,
    SCALAR* z_L,
    SCALAR* z_U,
    Ipopt::Index m,
    bool init_lambda,
    SCALAR* lambda)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering get_starting_point()" << std::endl;
#endif  //DEBUG_PRINT
    //
    if (init_x)
    {
        MapVecXs xVec(x, n);
        this->nlp_->getInitialGuess(n, xVec);
    }

    if (init_z)
    {
        MapVecXs z_lVec(z_L, n);
        MapVecXs z_uVec(z_U, n);
        this->nlp_->getBoundMultipliers(n, z_lVec, z_uVec);
    }

    if (init_lambda)
    {
        MapVecXs lambdaVec(lambda, m);
        this->nlp_->getLambdaVars(m, lambdaVec);
    }


#ifdef DEBUG_PRINT
    std::cout << "... entering get_starting_point()" << std::endl;
#endif  //DEBUG_PRINT

    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::eval_f(Ipopt::Index n, const SCALAR* x, bool new_x, SCALAR& obj_value)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering eval_f()" << std::endl;
#endif  //DEBUG_PRINT
    MapConstVecXs xVec(x, n);
    this->nlp_->extractOptimizationVars(xVec, new_x);
    obj_value = this->nlp_->evaluateCostFun();
    assert(obj_value == obj_value);

#ifdef DEBUG_PRINT
    std::cout << "... leaving eval_f()" << std::endl;
#endif  //DEBUG_PRINT
    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::eval_grad_f(Ipopt::Index n, const SCALAR* x, bool new_x, SCALAR* grad_f)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering eval_grad_f()" << std::endl;
#endif  //DEBUG_PRINT
    MapVecXs grad_fVec(grad_f, n);
    MapConstVecXs xVec(x, n);
    this->nlp_->extractOptimizationVars(xVec, new_x);
    this->nlp_->evaluateCostGradient(n, grad_fVec);

#ifdef DEBUG_PRINT
    std::cout << "... leaving eval_grad_f()" << std::endl;
#endif  //DEBUG_PRINT
    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::eval_g(Ipopt::Index n, const SCALAR* x, bool new_x, Ipopt::Index m, SCALAR* g)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering eval_g()" << std::endl;
#endif  //DEBUG_PRINT
    assert(m == static_cast<int>(this->nlp_->getConstraintsCount()));
    MapConstVecXs xVec(x, n);
    this->nlp_->extractOptimizationVars(xVec, new_x);
    MapVecXs gVec(g, m);
    this->nlp_->evaluateConstraints(gVec);


#ifdef DEBUG_PRINT
    std::cout << "gVec: " << gVec.transpose() << std::endl;
    std::cout << "... leaving eval_g()" << std::endl;
#endif  //DEBUG_PRINT

    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::eval_jac_g(Ipopt::Index n,
    const SCALAR* x,
    bool new_x,
    Ipopt::Index m,
    Ipopt::Index nele_jac,
    Ipopt::Index* iRow,
    Ipopt::Index* jCol,
    SCALAR* values)
{
    if (values == NULL)
    {
#ifdef DEBUG_PRINT
        std::cout << "... entering eval_jac_g, values == NULL" << std::endl;
#endif  //DEBUG_PRINT
        // set indices of nonzero elements of the jacobian
        Eigen::Map<Eigen::VectorXi> iRowVec(iRow, nele_jac);
        Eigen::Map<Eigen::VectorXi> jColVec(jCol, nele_jac);
        this->nlp_->getSparsityPatternJacobian(nele_jac, iRowVec, jColVec);


#ifdef DEBUG_PRINT
        std::cout << "... leaving eval_jac_g, values == NULL" << std::endl;
#endif  //DEBUG_PRINT
    }
    else
    {
#ifdef DEBUG_PRINT
        std::cout << "... entering eval_jac_g, values != NULL" << std::endl;
#endif  //DEBUG_PRINT
        MapVecXs valVec(values, nele_jac);
        MapConstVecXs xVec(x, n);
        this->nlp_->extractOptimizationVars(xVec, new_x);
        this->nlp_->evaluateConstraintJacobian(nele_jac, valVec);


#ifdef DEBUG_PRINT
        std::cout << "... leaving eval_jac_g, values != NULL" << std::endl;
#endif  //DEBUG_PRINT
    }

    return true;
}

template <typename SCALAR>
bool IpoptSolver<SCALAR>::eval_h(Ipopt::Index n,
    const SCALAR* x,
    bool new_x,
    SCALAR obj_factor,
    Ipopt::Index m,
    const SCALAR* lambda,
    bool new_lambda,
    Ipopt::Index nele_hess,
    Ipopt::Index* iRow,
    Ipopt::Index* jCol,
    SCALAR* values)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering eval_h()" << std::endl;
#endif
    if (values == NULL)
    {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.
        Eigen::Map<Eigen::VectorXi> iRowVec(iRow, nele_hess);
        Eigen::Map<Eigen::VectorXi> jColVec(jCol, nele_hess);
        this->nlp_->getSparsityPatternHessian(nele_hess, iRowVec, jColVec);
    }
    else
    {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only
        MapVecXs valVec(values, nele_hess);
        MapConstVecXs xVec(x, n);
        MapConstVecXs lambdaVec(lambda, m);
        this->nlp_->extractOptimizationVars(xVec, new_x);
        this->nlp_->evaluateHessian(nele_hess, valVec, obj_factor, lambdaVec);
    }

// ATTENTION: for hard coding of the Hessian, one only needs the lower left corner (since it is symmetric) - IPOPT knows that

#ifdef DEBUG_PRINT
    std::cout << "... leaving eval_h()" << std::endl;
#endif

    return true;
}

template <typename SCALAR>
void IpoptSolver<SCALAR>::finalize_solution(Ipopt::SolverReturn status,
    Ipopt::Index n,
    const SCALAR* x,
    const SCALAR* z_L,
    const SCALAR* z_U,
    Ipopt::Index m,
    const SCALAR* g,
    const SCALAR* lambda,
    SCALAR obj_value,
    const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq)
{
#ifdef DEBUG_PRINT
    std::cout << "... entering finalize_solution() ..." << std::endl;
#endif
    MapConstVecXs xVec(x, n);
    MapConstVecXs zLVec(z_L, n);
    MapConstVecXs zUVec(z_U, n);
    MapConstVecXs lambdaVec(lambda, m);

    this->nlp_->extractIpoptSolution(xVec, zLVec, zUVec, lambdaVec);
}
