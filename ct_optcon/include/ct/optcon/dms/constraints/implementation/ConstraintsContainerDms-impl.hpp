template <size_t STATE_DIM, size_t CONTROL_DIM>
ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::ConstraintsContainerDms(
		std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
		std::shared_ptr<TimeGrid> timeGrid,
		std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers,
		std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsIntermediate,
		std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsFinal,
		const state_vector_t& x0,
		const DmsSettings settings) 
	:
	settings_(settings),
	shotContainers_(shotContainers)
{
	constraintsCount_ = 0;

	// initial state constraint
	c_init_ = std::shared_ptr<InitStateConstraint<STATE_DIM, CONTROL_DIM>> (
			new InitStateConstraint<STATE_DIM, CONTROL_DIM> (x0, w));

	c_init_->initialize(constraintsCount_);
	constraintsCount_ += c_init_->getConstraintSize();
	c_lb_.conservativeResize(constraintsCount_);
	c_ub_.conservativeResize(constraintsCount_);
	c_init_->getLowerBound(c_lb_);
	c_init_->getUpperBound(c_ub_);

	constraints_.push_back(c_init_);

	// generate and initialize continuity constraints
	for(size_t shotNr = 0; shotNr < settings_.N_; shotNr++)
	{
		std::shared_ptr<ContinuityConstraint<STATE_DIM, CONTROL_DIM>> c_i = std::shared_ptr<ContinuityConstraint<STATE_DIM, CONTROL_DIM>> (
			new ContinuityConstraint<STATE_DIM, CONTROL_DIM>(shotContainers[shotNr], w, shotNr, settings));

		c_i->initialize(constraintsCount_);

		constraintsCount_ += c_i->getConstraintSize();
		c_lb_.conservativeResize(constraintsCount_);
		c_ub_.conservativeResize(constraintsCount_);

		c_i->getLowerBound(c_lb_);
		c_i->getUpperBound(c_ub_);

		constraints_.push_back(c_i);
	}


	// initialize terminal state constraint
	if(constraintsFinal)
	{
		constraintsFinal->initialize(constraintsCount_);
		constraintsCount_ += constraintsFinal->getConstraintSize();
		c_lb_.conservativeResize(constraintsCount_);
		c_ub_.conservativeResize(constraintsCount_);
		constraintsFinal->getLowerBound(c_lb_);
		constraintsFinal->getUpperBound(c_ub_);
		constraints_.push_back(constraintsFinal); 		
	}

	// if time is an optimization variable
	if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
	{
		// add the timehorizonequalityconstraint
		std::shared_ptr<TimeHorizonEqualityConstraint<STATE_DIM, CONTROL_DIM>> c_horizon_equal = 
			std::shared_ptr<TimeHorizonEqualityConstraint<STATE_DIM, CONTROL_DIM>> (
				new TimeHorizonEqualityConstraint<STATE_DIM, CONTROL_DIM> (w, timeGrid, settings));

		c_horizon_equal->initialize(constraintsCount_);

		constraintsCount_+= c_horizon_equal->getConstraintSize();
		c_lb_.conservativeResize(constraintsCount_);
		c_ub_.conservativeResize(constraintsCount_);
		c_horizon_equal->getLowerBound(c_lb_);
		c_horizon_equal->getUpperBound(c_ub_);

		constraints_.push_back(c_horizon_equal);
	}

	if(constraintsIntermediate)
	{
		constraintsIntermediate->initialize(constraintsCount_);
		constraintsCount_ += constraintsIntermediate->getConstraintSize();
		c_lb_.conservativeResize(constraintsCount_);
		c_ub_.conservativeResize(constraintsCount_);
		constraintsIntermediate->getLowerBound(c_lb_);
		constraintsIntermediate->getUpperBound(c_ub_);
		constraints_.push_back(constraintsIntermediate); 
	}

	nonZerosJacobianCount_ = 0;

	for(auto constraint : constraints_)
		nonZerosJacobianCount_ += constraint->getNumNonZerosJacobian();	

}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::prepareEvaluation()
{
	#pragma omp parallel for num_threads( settings_.nThreads_ )
	for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
		(*shotContainer)->integrateShot(settings_.dt_sim_);
	}	
}

// template <size_t STATE_DIM, size_t CONTROL_DIM>
// void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::evalConstraints(Eigen::Map<Eigen::VectorXd>& c_local)
// {
// 	c_local.setConstant(0.0); 	// todo: remove?
// 	size_t count = 0;

// 	#pragma omp parallel for num_threads( settings_.nThreads_ )
// 	for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
// 		(*shotContainer)->integrateShot(settings_.dt_sim_);
// 	}

// 	for(auto constraint : constraints_)
// 		count = constraint->getEvaluation(c_local, count);

// }


// template <size_t STATE_DIM, size_t CONTROL_DIM>
// void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::getSparsityPattern(
// 		Eigen::Map<Eigen::VectorXi>& iRow_vec,
// 		Eigen::Map<Eigen::VectorXi>& jCol_vec,
// 		const int nnz_jac_g)
// {
// 	size_t count = 0;
	
// 	for(auto constraint : constraints_)
// 		count = constraint->genSparsityPattern(iRow_vec, jCol_vec, count);	

// 	assert(count == (size_t) nnz_jac_g); //ensure we have entered the right number of elements.
// }
// 
template <size_t STATE_DIM, size_t CONTROL_DIM>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::prepareJacobianEvaluation()
{	
	#pragma omp parallel for num_threads( settings_.nThreads_ )
	for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
		(*shotContainer)->integrateShotandComputeSensitivity();
	}
}



// IMPORTANT: this function must use the same indexing functionality as the pattern generator above.
// template <size_t STATE_DIM, size_t CONTROL_DIM>
// void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::evalSparseJacobian(Eigen::Map<Eigen::VectorXd>& val, const int nzz_jac_g)
// {
// 	val.setConstant(0.0); 	// todo: remove?
// 	size_t count = 0;
	
// 	#pragma omp parallel for num_threads( settings_.nThreads_ )
// 	for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
// 		(*shotContainer)->integrateShotandComputeSensitivity();
// 	}

// 	for(auto constraint : constraints_)
// 		count = constraint->evalConstraintJacobian(val, count);	

// 	assert(count == (size_t) nzz_jac_g); //ensure we have evaluated the right number of elements.
// }

template <size_t STATE_DIM, size_t CONTROL_DIM>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>::updateInitialConstraint(const state_vector_t& x_init_new)
{
	c_init_->updateConstraint(x_init_new);
}
