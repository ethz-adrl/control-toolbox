// constructor
IpoptSolver::IpoptSolver(std::shared_ptr<Nlp> nlp, const NlpSolverSettings& settings) :
		BASE(nlp, settings),
		settings_(BASE::settings_.ipoptSettings_)
{
	//Constructor arguments
	//Argument 1: create console output
	//Argument 2: create empty
	ipoptApp_ = std::shared_ptr<Ipopt::IpoptApplication> (new Ipopt::IpoptApplication(true,false));
}

//destructor
IpoptSolver::~IpoptSolver()
{
	// Needed to destruct all the IPOPT memory
	Ipopt::Referencer* t=NULL;
	this->ReleaseRef(t);
	delete t;
}

void IpoptSolver::configureDerived(const NlpSolverSettings& settings)
{
	std::cout << "calling Ipopt configure derived" << std::endl;
	settings_ = settings.ipoptSettings_;
	setSolverOptions();
	isInitialized_ = true;
}

void IpoptSolver::setSolverOptions() {
	// if(isInitialized_ == true)
	// 	throw std::runtime_error("NLP is already initialized, but initialized method gets called again: something's wrong!");

	ipoptApp_->Options()->SetNumericValue("tol", settings_.tol_);
	ipoptApp_->Options()->SetNumericValue("constr_viol_tol", settings_.constr_viol_tol_);
	// ipoptApp_->Options()->SetNumericValue("constr_viol_tol", 1e-3);
	// ipoptApp_->Options()->SetIntegerValue("max_iter", 10);
			// ipoptApp_->Options()->SetNumericValue("resto.tol", 1e-7);
			// ipoptApp_->Options()->SetNumericValue("acceptable_tol", 1e-7);
			// ipoptApp_->Options()->SetNumericValue("resto.acceptable_tol", 1e-7);
	ipoptApp_->Options()->SetStringValueIfUnset("linear_scaling_on_demand", settings_.linear_scaling_on_demand_);
	ipoptApp_->Options()->SetStringValueIfUnset("hessian_approximation", settings_.hessian_approximation_);
	//ipoptApp_->Options()->SetStringValueIfUnset("nlp_scaling_method", "gradient-based");
	ipoptApp_->Options()->SetIntegerValue("print_level", settings_.printLevel_); //working now
	ipoptApp_->Options()->SetStringValueIfUnset("print_user_options", settings_.print_user_options_);
	// ipoptApp_->Options()->SetNumericValue("print_frequency_iter", 10000);
	//ipoptApp_->Options()->SetStringValueIfUnset("print_info_string", "yes");
	ipoptApp_->Options()->SetStringValueIfUnset("derivative_test", settings_.derivativeTest_);

	ipoptApp_->Options()->SetIntegerValue("print_level", settings_.printLevel_);

	ipoptApp_->Options()->SetNumericValue("derivative_test_tol", settings_.derivativeTestTol_);
	ipoptApp_->Options()->SetNumericValue("derivative_test_perturbation", settings_.derivativeTestPerturbation_);
	ipoptApp_->Options()->SetNumericValue("point_perturbation_radius", settings_.point_perturbation_radius_);
	// ipoptApp_->Options()->SetStringValueIfUnset("check_derivatives_for_naninf", "yes");
	//ipoptApp_->Options()->SetStringValueIfUnset("derivative_test_print_all", "yes");
	ipoptApp_->Options()->SetStringValueIfUnset("linear_system_scaling", settings_.linearSystemScaling_);
	ipoptApp_->Options()->SetStringValueIfUnset("linear_solver", settings_.linear_solver_);

	// ipoptApp_->Options()->SetStringValueIfUnset("jacobian_approximation", "finite-difference-values");
}

bool IpoptSolver::solve() {

	// Initialize the problem
	status_ = ipoptApp_->Initialize();

	if (status_ == Ipopt::Solve_Succeeded)
	{
		std::cout << std::endl << std::endl
				<< "*** Initialized successfully -- starting NLP." << std::endl;
	}
	else
	{
		throw(std::runtime_error("NLP initialization failed"));
	}

	// Ask Ipopt to solve the problem
	status_ = ipoptApp_->OptimizeTNLP(this);

	if (status_ == Ipopt::Solve_Succeeded || status_ == Ipopt::Solved_To_Acceptable_Level) {
		// Retrieve some statistics about the solve
		Ipopt::Index iter_count = ipoptApp_->Statistics()->IterationCount();
		std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

		Number final_obj = ipoptApp_->Statistics()->FinalObjective();
		std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
		return true;
	}
	else{
		std::cout << " ipopt return value: " << status_ << std::endl;
		return false;
	}
}

void IpoptSolver::prepareWarmStart(size_t maxIterations)
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

bool IpoptSolver::solveSucceeded()
{
	if(status_ == Ipopt::Solve_Succeeded || status_ == Ipopt::Solved_To_Acceptable_Level || status_ == Ipopt::Maximum_Iterations_Exceeded)
		return true;
	else
		return false;
}


/**
 * returns the size of a problem
 * @param n				dimension of optimization variable vector x
 * @param m				number of constraints (liniear + nonlinear) without bounds on x
 * @param nnz_jac_g		number of nonzero elements in jacobian of the constraints
 * @param nnz_h_lag		unused
 * @param index_style	index style (starting indices at 0 or at 1?)
 * @return
 */
bool IpoptSolver::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
		Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	// dimension of optimization variable vector x
	n = nlp_->getVarCount();
	assert(n == n);

	// number of constraints (liniear + nonlinear) without bounds on x
	m = nlp_->getConstraintsCount();
	assert(m == m);

	// number of nonzero elements in jacobian of the constraints
	nnz_jac_g = nlp_->getNonZeroJacobianCount();
	assert(nnz_jac_g==nnz_jac_g);

	// use the C style indexing (0-based)
	index_style = Ipopt::TNLP::C_STYLE;

#ifdef DEBUG_PRINT
	std::cout << "... number of decision variables = " << n << std::endl;
	std::cout << "... number of constraints = " << m << std::endl;
	std::cout << "... nonzeros in jacobian = " << nnz_jac_g << std::endl;
#endif

	return true;
}


// returns the variable bounds
bool IpoptSolver::get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u,
		Ipopt::Index m, Number* g_l, Number* g_u)
{
#ifdef DEBUG_PRINT
	std::cout << "... entering get_bounds_info()" << std::endl;
#endif //DEBUG_PRINT
	MapVecXd x_lVec(x_l, n);
	MapVecXd x_uVec(x_u, n);
	MapVecXd g_lVec(g_l, m);
	MapVecXd g_uVec(g_u, m);
	nlp_->getVariableBounds(x_lVec, x_uVec, n);
	// bounds on optimization vector
	// x_l <= x <= x_u
	assert(x_l == x_l);
	assert(x_u == x_u);
	assert(n == n);

	// constraints bounds (e.g. for equality constraints = 0)
	nlp_->getConstraintBounds(g_lVec, g_uVec, m);
	assert(g_l == g_l);
	assert(g_u == g_u);

#ifdef DEBUG_PRINT
	std::cout << "... Leaving get_bounds_info()" << std::endl;
#endif //DEBUG_PRINT

	return true;
}


// returns the initial point for the problem
bool IpoptSolver::get_starting_point(Ipopt::Index n, bool init_x, Number* x,
		bool init_z, Number* z_L, Number* z_U,
		Ipopt::Index m, bool init_lambda,
		Number* lambda)
{
	#ifdef DEBUG_PRINT
	std::cout << "... entering get_starting_point()" << std::endl;
#endif //DEBUG_PRINT
	// We should try these with warmstart
	if(init_x)
	{
		MapVecXd xVec(x, n);
		nlp_->getPrimalVars(n, xVec);
	}

	if(init_z)
	{
		MapVecXd z_lVec(z_L, n);
		MapVecXd z_uVec(z_U, n);
		nlp_->getInitBoundMultipliers(n, z_lVec, z_uVec);
	}

	if(init_lambda)
	{
		MapVecXd lambdaVec(lambda, m);
		nlp_->getInitLambdaVars(m, lambdaVec);
	}

	// for(size_t i = 0; i< n; ++i)
	// 	std::cout << "xstart IPOPT: " << x[i] << std::endl;
	#ifdef DEBUG_PRINT
	std::cout << "... entering get_starting_point()" << std::endl;
#endif //DEBUG_PRINT

	return true;
}


// returns the value of the objective function
bool IpoptSolver::eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value)
{
#ifdef DEBUG_PRINT
	std::cout << "... entering eval_f()" << std::endl;
#endif //DEBUG_PRINT
	MapConstVecXd xVec(x, n);
	nlp_->setPrimalVars(xVec, new_x);
	obj_value = nlp_->evaluateCostFun();
	// std::cout << "F IPOPT: " << obj_value << std::endl;
	assert(obj_value == obj_value);

#ifdef DEBUG_PRINT
	std::cout << "... leaving eval_f()" << std::endl;
#endif //DEBUG_PRINT
	return true;
}


// return the gradient of the objective function grad_x(f(x))
bool IpoptSolver::eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f)
{
#ifdef DEBUG_PRINT
	std::cout << "... entering eval_grad_f()" << std::endl;
#endif //DEBUG_PRINT
	MapVecXd grad_fVec(grad_f, n);
	MapConstVecXd xVec(x, n);
	nlp_->setPrimalVars(xVec, new_x);
	nlp_->evaluateCostGradient(n, grad_fVec);

#ifdef DEBUG_PRINT
	std::cout << "... leaving eval_grad_f()" << std::endl;
#endif //DEBUG_PRINT
	return true;
}


// return the value of the constraints: g(x)
bool IpoptSolver::eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g)
{
#ifdef DEBUG_PRINT
	std::cout << "... entering eval_g()" << std::endl;
#endif //DEBUG_PRINT
	assert(m == nlp_->getConstraintsCount());
	MapConstVecXd xVec(x, n);
	nlp_->setPrimalVars(xVec, new_x);
	MapVecXd gVec(g, m);
	nlp_->evaluateConstraints(gVec);


#ifdef DEBUG_PRINT
	std::cout << "gVec: " << gVec.transpose() << std::endl;
	std::cout << "... leaving eval_g()" << std::endl;
#endif //DEBUG_PRINT

	return true;
}


// return the structure or values of the jacobian
bool IpoptSolver::eval_jac_g(Ipopt::Index n, const Number* x, bool new_x,
			Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
			Number* values)
{
		// MatlabInterface mi("/home/markusta/Documents/Code/catkin_ws/src/ct/ct_optcon/test/dms/oscillator/matfiles/sparsityPattern.mat");
	if (values == NULL)
	{
#ifdef DEBUG_PRINT
	std::cout << "... entering eval_jac_g, values == NULL" << std::endl;
#endif //DEBUG_PRINT
		// set indices of nonzero elements of the jacobian
		Eigen::Map<Eigen::VectorXi> iRowVec(iRow, nele_jac);
		Eigen::Map<Eigen::VectorXi> jColVec(jCol, nele_jac);
		nlp_->getSparsityPattern(nele_jac, iRowVec, jColVec);


#ifdef DEBUG_PRINT
		// std::cout << "nele_jac: " << nele_jac << std::endl;
		// std::cout << "iRowVec size: " << iRowVec.rows() << std::endl;
		// std::cout << "iRowVec: " << iRowVec.transpose() << std::endl;
		// std::cout << "jCol size: " << jColVec.rows() << std::endl;
		// std::cout << "jCol: " << jColVec.transpose() << std::endl;
		// mi.sendScalarTrajectoryToMatlab(iRowVec, "iRow");
		// mi.sendScalarTrajectoryToMatlab(jColVec, "jCol");


	std::cout << "... leaving eval_jac_g, values == NULL" << std::endl;
#endif //DEBUG_PRINT
	}
	else
	{
#ifdef DEBUG_PRINT
	std::cout << "... entering eval_jac_g, values != NULL" << std::endl;
#endif //DEBUG_PRINT
		MapVecXd valVec(values, nele_jac);
		MapConstVecXd xVec(x, n);
		nlp_->setPrimalVars(xVec, new_x);
		nlp_->evaluateConstraintJacobian(nele_jac, valVec);


		// std::cout << "valVec: " << valVec.transpose() << std::endl;
#ifdef DEBUG_PRINT
		mi.sendScalarTrajectoryToMatlab1(valVec, "val");
	std::cout << "... leaving eval_jac_g, values != NULL" << std::endl;
#endif //DEBUG_PRINT
	}

	return true;
}


//return the structure or values of the hessian
bool IpoptSolver::eval_h(Ipopt::Index n, const Number* x, bool new_x,
			Number obj_factor, Ipopt::Index m, const Number* lambda,
			bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
			Ipopt::Index* jCol, Number* values)
{
	std::cerr << "should not be evaluating hessian" << std::endl;

#ifdef DEBUG_PRINT
	std::cout << "... entering eval_h()" << std::endl;
#endif
	if (values == NULL)
	{
		// return the structure. This is a symmetric matrix, fill the lower left
		// triangle only.
	}
	else
	{
		// return the values. This is a symmetric matrix, fill the lower left
		// triangle only
	}

	// only needed if quasi-newton approximation is not used, hence set to -1 (not used)!
	// ATTENTION: for hard coding of the hessian, one only needs the lower left corner (since it is symmetric) - IPOPT knows that
	//nnz_h_lag = -1;

#ifdef DEBUG_PRINT
	std::cout << "... leaving eval_h()" << std::endl;
#endif

	return true;
}

void IpoptSolver::finalize_solution(Ipopt::SolverReturn status,
		Ipopt::Index n, const Number* x, const Number* z_L, const Number* z_U,
		Ipopt::Index m, const Number* g, const Number* lambda,
		Number obj_value,
		const Ipopt::IpoptData* ip_data,
		Ipopt::IpoptCalculatedQuantities* ip_cq)
{
#ifdef DEBUG_PRINT
	std::cout << "... entering finalize_solution() ..." << std::endl;
#endif
	MapConstVecXd xVec(x, n);
	MapConstVecXd zLVec(z_L, n);
	MapConstVecXd zUVec(z_U, n);
	MapConstVecXd lambdaVec(lambda, m);

	nlp_->extractSolution(xVec, zLVec, zUVec, lambdaVec);
}

