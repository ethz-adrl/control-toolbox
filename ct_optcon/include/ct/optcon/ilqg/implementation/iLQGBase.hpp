/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::setInitialGuess(const Policy_t& initialGuess)
{
	if(initialGuess.K().size() != initialGuess.uff().size())
	{
		std::cout << "Provided initial feedforward and feedback controllers are not equally long.";
		std::cout << "Feedforward length is "<<initialGuess.uff().size()<<" but feedback length is "<<initialGuess.K().size()<<std::endl;
		throw(std::runtime_error("Feedforward and feedback controllers not equally long"));
	}

	if(initialGuess.K().size() < K_){
		std::cout << "Initial guess length too short. Received length " << initialGuess.K().size() <<", expected " << K_ << std::endl;
		throw std::runtime_error("initial control guess to short");
	}

	if(initialGuess.K().size() > K_)
		std::cout << "Warning, initial control guess too long, will truncate" << std::endl;

	u_ff_ = ControlVectorArray(initialGuess.uff().begin(), initialGuess.uff().begin()+K_);
	L_ = FeedbackArray(initialGuess.K().begin(), initialGuess.K().begin()+K_);

	initialized_ = true;

	reset();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::changeTimeHorizon(const SCALAR& tf)
{
	if (tf < 0)
		throw std::runtime_error("negative time horizon specified");

	int K = settings_.computeK(tf);

	if (K == K_) return;
	K_ = K;

	A_.resize(K_);
	B_.resize(K_);
	x_.resize(K_+1);
	u_.resize(K_);
	u_ff_.resize(K_);
	u_ff_prev_.resize(K_);
	gv_.resize(K_);
	G_.resize(K_);
	H_.resize(K_);
	Hi_.resize(K_);
	Hi_inverse_.resize(K_);
	lv_.resize(K_);
	L_.resize(K_);
	P_.resize(K_);
	q_.resize(K_+1);
	qv_.resize(K_+1);
	Q_.resize(K_+1);
	rv_.resize(K_);
	R_.resize(K_);
	sv_.resize(K_+1);
	S_.resize(K_+1);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0)
{
	if (x_.size() == 0)
		x_.resize(1);

	x_[0] = x0;
	reset(); // since initial state changed, we have to start fresh, i.e. with a rollout
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf)
{
	if (cf == nullptr)
		throw std::runtime_error("cost function is nullptr");

	this->getCostFunctionInstances().resize(settings_.nThreads+1);

	for (size_t i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		this->getCostFunctionInstances()[i] = typename Base::OptConProblem_t::CostFunctionPtr_t(cf->clone());
	}

	// recompute cost if line search is active
	if (iteration_ > 0 && settings_.lineSearchSettings.active)
		computeQuadraticCostsAroundTrajectory();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn)
{
	if (dyn == nullptr)
		throw std::runtime_error("system dynamics are nullptr");

	this->getNonlinearSystemsInstances().resize(settings_.nThreads+1);
	integratorsRK4_.resize(settings_.nThreads+1);
	integratorsEuler_.resize(settings_.nThreads+1);
	integratorsEulerSymplectic_.resize(settings_.nThreads+1);
	integratorsRkSymplectic_.resize(settings_.nThreads+1);

	for (size_t i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		this->getNonlinearSystemsInstances()[i] = typename Base::OptConProblem_t::DynamicsPtr_t(dyn->clone());
		this->getNonlinearSystemsInstances()[i]->setController(controller_[i]);

		if(controller_[i] == nullptr)
			throw std::runtime_error("Controller not defined");

		integratorsRK4_[i] = std::shared_ptr<ct::core::IntegratorRK4<STATE_DIM, SCALAR> > (new ct::core::IntegratorRK4<STATE_DIM, SCALAR>(this->getNonlinearSystemsInstances()[i]));
		integratorsEuler_[i] = std::shared_ptr<ct::core::IntegratorEuler<STATE_DIM, SCALAR> >(new ct::core::IntegratorEuler<STATE_DIM, SCALAR>(this->getNonlinearSystemsInstances()[i]));
		if(this->getNonlinearSystemsInstances()[i]->isSymplectic())
		{
			integratorsEulerSymplectic_[i] = std::shared_ptr<ct::core::IntegratorSymplecticEuler<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>>(
									new ct::core::IntegratorSymplecticEuler<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>(
										std::static_pointer_cast<ct::core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>> (this->getNonlinearSystemsInstances()[i])));
			integratorsRkSymplectic_[i] = std::shared_ptr<ct::core::IntegratorSymplecticRk<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>>(
									new ct::core::IntegratorSymplecticRk<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>(
										std::static_pointer_cast<ct::core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>> (this->getNonlinearSystemsInstances()[i])));
		}
	}
	reset(); // since system changed, we have to start fresh, i.e. with a rollout
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin)
{
	this->getLinearSystemsInstances().resize(settings_.nThreads+1);

	for (size_t i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		this->getLinearSystemsInstances()[i] = typename Base::OptConProblem_t::LinearPtr_t(lin->clone());
	}
	// technically a linear system change does not require a new rollout. Hence, we do not reset.
}




template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::checkProblem()
{
	if (K_==0)
		throw std::runtime_error("Time horizon too small resulting in 0 iLQG steps");

	if (L_.size() < K_)
	{
		std::cout << "Provided initial feedback controller too short, should be at least "<<K_<<" but is "<<L_.size()<<" long."<<std::endl;
		throw(std::runtime_error("Provided initial feedback controller too short"));
	}

	if (u_ff_.size() < K_)
	{
		std::cout << "Provided initial feed forward controller too short, should be at least "<<K_<<" but is " << u_ff_.size() <<" long."<<std::endl;
		throw(std::runtime_error("Provided initial feed forward controller too short"));
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::solve()
{
	bool foundBetter = true;
	size_t numIterations = 0;

	try{
		while (foundBetter && numIterations < settings_.max_iterations)
		{
#ifdef DEBUG_PRINT
			std::cout << "running iteration: " << numIterations+1 << std::endl;
#endif //DEBUG_PRINT
			foundBetter = runIteration();

			numIterations++;
		}
	}
	catch(std::exception& e){
		std::cout << "iLQG solve() did not succeed due to: " << e.what() << std::endl;
		return false;
	}

	return (numIterations > 1 || foundBetter || (numIterations == 1 && !foundBetter));
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::configure(
	const iLQGSettings& settings)
{
	if (!settings.parametersOk())
	{
		throw(std::runtime_error("iLQGSettings are incorrect. Aborting."));
	}

	if (settings.nThreads != settings_.nThreads)
	{
		throw(std::runtime_error("Number of threads at iLQG cannot be changed after instance has been created."));
	}

	// will be set correctly later
	Eigen::setNbThreads(settings.nThreadsEigen);

	settings_ = settings;

	H_corrFix_ = settings_.epsilon*ControlMatrix::Identity();

	reset();

	configured_ = true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::runIteration()
{
	if (!initialized_)
		throw std::runtime_error("iLQG is not initialized!");

	if (!configured_)
		throw std::runtime_error("iLQG is not configured!");

	smallestEigenvalueIteration_ = std::numeric_limits<scalar_t>::infinity();

	checkProblem();

#ifdef DEBUG_PRINT
	std::cout<<"[iLQG]: #1 ForwardPass"<<std::endl;
#endif // DEBUG_PRINT
	auto start = std::chrono::steady_clock::now();
	auto startEntire = start;
	if (!forwardPass())
	{
#ifdef DEBUG_PRINT
		std::cout<<"[iLQG]: System became unstable, aborting iteration."<<std::endl;
#endif // DEBUG_PRINT
		return false;
	}
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	//std::cout << "Forward pass took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;

#ifdef DEBUG_PRINT
	std::cout<<"[iLQG]: #2 BackwardPass"<<std::endl;
#endif // DEBUG_PRINT
	start = std::chrono::steady_clock::now();
	backwardPass();
	end = std::chrono::steady_clock::now();
	diff = end - start;
#ifdef DEBUG_PRINT
	std::cout << "Backward pass took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

#ifdef DEBUG_PRINT
	std::cout<<"[iLQG]: #3 LineSearch"<<std::endl;
#endif // DEBUG_PRINT

	start = std::chrono::steady_clock::now();
	bool foundBetter = lineSearchController();
	end = std::chrono::steady_clock::now();
	diff = end - start;
#ifdef DEBUG_PRINT
	std::cout << "Line search took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

	if (settings_.nThreadsEigen > 1)
		Eigen::setNbThreads(settings_.nThreadsEigen); // restore default Eigen thread number

	diff = end - startEntire;
#ifdef DEBUG_PRINT
	std::cout << "Total iteration took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

#ifdef DEBUG_PRINT
	debugPrint();
#endif //DEBUG_PRINT

#ifdef MATLAB_FULL_LOG
	logToMatlab();
#endif //MATLAB_FULL_LOG

	iteration_++;

	return foundBetter;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const typename iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::Policy_t& iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::getSolution()
{
	TimeArray t_temp = t_;
	t_temp.pop_back();

	policy_.update(u_ff_, L_, t_temp);

	return policy_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::retrieveLastLinearizedModel(StateMatrixArray& A, StateControlMatrixArray& B)
{
	// todo fix me!
	A = A_;
	B = B_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::rolloutSystem (
		size_t threadId,
		const ControlVectorArray& u_ff_local,
		ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
		ct::core::tpl::TimeArray<SCALAR>& t_local,
		std::atomic_bool* terminationFlag) const
{
	const scalar_t& dt = settings_.dt;
	const scalar_t& dt_sim = settings_.dt_sim;
	const size_t K_local = K_;

	// take a copy since x0 gets overwritten in integrator
	ct::core::StateVector<STATE_DIM, SCALAR> x0 = x_local[0];

	// compute number of substeps
	size_t steps = round(dt/ dt_sim);

	x_local.clear();
	t_local.clear();
	u_local.clear();

	x_local.push_back(x0);
	t_local.push_back(0.0);

	for (size_t i = 0; i<K_local; i++)
	{
		if (terminationFlag && *terminationFlag) return false;

		u_local.push_back( u_ff_local[i] + L_[i] * x0);
		controller_[threadId]->setControl(u_local.back());

		for (size_t j=0; j<steps; j++)
		{
			if (steps > 1)
			{
				//controller_[threadId]->u() = (u_ff_[threadId][i] + L_[i]*x0);
			}

			if (settings_.integrator == iLQGSettings::EULER)
			{
				integratorsEuler_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
			}
			else if(settings_.integrator == iLQGSettings::RK4)
			{
				integratorsRK4_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
			}
			else if(settings_.integrator == iLQGSettings::EULER_SYM)
			{
				integratorsEulerSymplectic_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
			}
			else if(settings_.integrator == iLQGSettings::RK_SYM)
			{
				integratorsRkSymplectic_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
			}
			else
				throw std::runtime_error("invalid integration mode selected.");
		}

		x_local.push_back(x0);
		t_local.push_back((i+1)*dt_sim);

		// check if nan
		for (size_t k=0; k<STATE_DIM; k++)
		{
			if (isnan(x_local[i](k)))
			{
				return false;
			}
		}
		for (size_t k=0; k<CONTROL_DIM; k++)
		{
			if (isnan(u_local[i](k)))
			{
				std::cout << "control unstable" << std::endl;
				return false;
			}
		}
	}

	if(x_local.size() != K_local+1) {
		std::cout << "Error: Rollout did not provide the correct amount of states. Should have been "<<K_+1<<" but was "<<x_local.size()<<std::endl;
		throw std::runtime_error("Error: Dynamics did not provide the correct amount of states.");
	}

	if(u_local.size() != K_local) {
		std::cout << "Error: Rollout did not provide the correct amount of controls. Should have been "<<K_<<" but was "<<u_local.size()<<std::endl;
		throw std::runtime_error("Error: Dynamics did not provide the correct amount of controls.");
	}

	return true;
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::forwardPass()
{
	// Forward pass according to section V. Summary of the Algorithm of the iLQGMP paper

	// step 1
	// calculates x_k+1 = x_k + dt * f(x_k,u_k)
	if (iteration_ == 0)
	{
		if (!rolloutSystem(settings_.nThreads, u_ff_, x_, u_, t_))
			throw std::runtime_error("Rollout failed. System became unstable");
	}

	createLQProblem();

	if (settings_.nThreadsEigen > 1)
		Eigen::setNbThreads(settings_.nThreadsEigen); // restore default Eigen thread number

	return true;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::sequentialLQProblem()
{
	auto start = std::chrono::steady_clock::now();
	computeLinearizedDynamicsAroundTrajectory();
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
#ifdef DEBUG_PRINT
	std::cout << "Linearizing dynamics took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

	start = std::chrono::steady_clock::now();
	computeQuadraticCostsAroundTrajectory();
	end = std::chrono::steady_clock::now();
	diff = end - start;
#ifdef DEBUG_PRINT
	std::cout << "Cost computation took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::lineSearchController()
{
	// if first iteration, we have to find cost of initial rollout
	if (iteration_ == 0)
	{
		intermediateCostBest_ = 0.0;

		for (int k=K_-1; k>=0; k--)
			intermediateCostBest_ += q_[k];

		finalCostBest_ = q_[K_];
	}

	// lowest cost is cost of last rollout
	lowestCost_ = intermediateCostBest_ + finalCostBest_;
	scalar_t lowestCostPrevious = lowestCost_;

	if (!settings_.lineSearchSettings.active)
	{
		ControlVectorArray u_ff_local(K_);
		TimeArray t_local(K_);

		for (int k=K_-1; k>=0; k--)
		{
			u_ff_local[k] = lv_[k] - L_[k] * x_[k] + u_[k];
		}

		bool dynamicsGood = rolloutSystem(settings_.nThreads, u_ff_local, x_, u_, t_local);

		if (dynamicsGood)
		{
			intermediateCostBest_ = std::numeric_limits<scalar_t>::max();
			finalCostBest_ = std::numeric_limits<scalar_t>::max();
			computeCostsOfTrajectory(settings_.nThreads, x_, u_, intermediateCostBest_, finalCostBest_);
			lowestCost_ = intermediateCostBest_ + finalCostBest_;
			u_ff_.swap(u_ff_local);
			t_.swap(t_local);
		}
		else
		{
#ifdef DEBUG_PRINT
std::cout<<"CONVERGED: System became unstable!" << std::endl;
#endif //DEBUG_PRINT
			return false;
		}
	} else
	{
		// compute the feedforward control action that leads to the current trajectory
		for (int k=K_-1; k>=0; k--) {
			u_ff_prev_[k] = - L_[k] * x_[k] + u_[k];
		}

#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Starting line search."<<std::endl;
		std::cout<<"[LineSearch]: Cost last rollout: "<<lowestCost_<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

		scalar_t alphaBest = performLineSearch();

#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Best control found at alpha: "<<alphaBest<<" . Will use this control."<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

#ifdef DEBUG_PRINT
		if (alphaBest == 0.0)
		{
			std::cout<<"WARNING: No better control found. Converged."<<std::endl;
			return false;
		}
#endif

		if (settings_.lineSearchSettings.adaptive)
		{
			// learning alpha
			#ifdef DEBUG_PRINT_LINESEARCH
					std::cout<<"[LineSearch]: Adapting alpha. Best alpha: "<<alphaBest<<". Current alpha0: "<<settings_.lineSearchSettings.alpha_0<<std::endl;
			#endif //DEBUG_PRINT_LINESEARCH
			settings_.lineSearchSettings.alpha_0 = learnAlpha(alphaBest);
			#ifdef DEBUG_PRINT_LINESEARCH
					std::cout<<"[LineSearch]: Adapted new alpha: "<<settings_.lineSearchSettings.alpha_0<<std::endl;
			#endif //DEBUG_PRINT_LINESEARCH
		}
	}

	if ((lowestCostPrevious - lowestCost_)/lowestCostPrevious > settings_.min_cost_improvement)
	{
		return true;
	}

#ifdef DEBUG_PRINT
	std::cout<<"CONVERGED: Cost last iteration: "<<lowestCostPrevious<<", current cost: "<< lowestCost_ << std::endl;
	std::cout<<"CONVERGED: Cost improvement ratio was: "<<(lowestCostPrevious - lowestCost_)/lowestCostPrevious <<"x, which is lower than convergence criteria: "<<settings_.min_cost_improvement<<std::endl;
#endif //DEBUG_PRINT
	return false;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::learnAlpha(const SCALAR& alphaBest)
{
	scalar_t alphaNew = alphaBest;

	if (alphaBest < settings_.lineSearchSettings.alpha_max)
	{
		// start with alpha one higher than the best one
		alphaNew /= settings_.lineSearchSettings.n_alpha;
	}
#ifdef DEBUG_PRINT_LINESEARCH
	std::cout<<"[LineSearch]: Learning alpha. New alpha0: "<<alphaNew<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

	return alphaNew;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::lineSearchSingleController(
		size_t threadId,
		scalar_t alpha,
		ControlVectorArray& u_ff_local,
		ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
		ct::core::tpl::TimeArray<SCALAR>& t_local,
		scalar_t& intermediateCost,
		scalar_t& finalCost,
		std::atomic_bool* terminationFlag
) const
{
	intermediateCost = std::numeric_limits<scalar_t>::max();
	finalCost = std::numeric_limits<scalar_t>::max();

	if (terminationFlag && *terminationFlag) return;

	for (int k=K_-1; k>=0; k--)
	{
		u_ff_local[k] = alpha * lv_[k] + u_ff_prev_[k];
	}

	bool dynamicsGood = rolloutSystem(threadId, u_ff_local, x_local, u_local, t_local, terminationFlag);

	if (terminationFlag && *terminationFlag) return;

	if (dynamicsGood)
	{
		computeCostsOfTrajectory(threadId, x_local, u_local, intermediateCost, finalCost);
	}
	else
	{
		std::string msg = std::string("dynamics not good, thread: ") + std::to_string(threadId);
		std::cout << msg << std::endl;
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::computeCostsOfTrajectory(
		size_t threadId,
		const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
		const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
		scalar_t& intermediateCost,
		scalar_t& finalCost
) const
{
	intermediateCost = 0;

	for (size_t k=0; k<K_; k++) {
		// feed current state and control to cost function
		this->getCostFunctionInstances()[threadId]->setCurrentStateAndControl(x_local[k], u_local[k], settings_.dt*k);

		// derivative of cost with respect to state
		intermediateCost += this->getCostFunctionInstances()[threadId]->evaluateIntermediate();
	}
	intermediateCost *= settings_.dt;

	this->getCostFunctionInstances()[threadId]->setCurrentStateAndControl(x_local[K_], control_vector_t::Zero(), settings_.dt*K_);
	finalCost = this->getCostFunctionInstances()[threadId]->evaluateTerminal();
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::computeLinearizedDynamics(size_t threadId, size_t k)
{
	switch(settings_.discretization)
	{
		case iLQGSettings::FORWARD_EULER:
		{
			A_[k] = state_matrix_t::Identity();
			A_[k] += settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k], u_[k], k*settings_.dt);
			B_[k] = settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k], u_[k], k*settings_.dt);
			break;
		}
		case iLQGSettings::BACKWARD_EULER:
		{
			state_matrix_t aNew = settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k], u_[k], k*settings_.dt);
			state_matrix_t aNewInv = (state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
			A_[k] = aNewInv;
			B_[k] = aNewInv * settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k], u_[k], k*settings_.dt);
			break;
		}
		case iLQGSettings::TUSTIN:
		{
			state_matrix_t aNew = 0.5 * settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k], u_[k], k*settings_.dt);
			state_matrix_t aNewInv = (state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
			A_[k] = aNewInv * (state_matrix_t::Identity() + aNew);
			B_[k] = aNewInv * settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k], u_[k], k*settings_.dt);
			break;
		}
		default:
		{
			throw std::runtime_error("Unknown discretization scheme");
			break;
		}
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::computeQuadraticCosts(size_t threadId, size_t k)
{
	const scalar_t& dt = settings_.dt;

	// feed current state and control to cost function
	this->getCostFunctionInstances()[threadId]->setCurrentStateAndControl(x_[k], u_[k], dt*k);

	// derivative of cost with respect to state
	q_[k] = this->getCostFunctionInstances()[threadId]->evaluateIntermediate()*dt;

	qv_[k] = this->getCostFunctionInstances()[threadId]->stateDerivativeIntermediate()*dt;

	Q_[k] = this->getCostFunctionInstances()[threadId]->stateSecondDerivativeIntermediate()*dt;

	// derivative of cost with respect to control and state
	P_[k] = this->getCostFunctionInstances()[threadId]->stateControlDerivativeIntermediate()*dt;

	// derivative of cost with respect to control
	rv_[k] = this->getCostFunctionInstances()[threadId]->controlDerivativeIntermediate()*dt;

	R_[k] = this->getCostFunctionInstances()[threadId]->controlSecondDerivativeIntermediate()*dt;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::initializeCostToGo()
{
	// feed current state and control to cost function
	this->getCostFunctionInstances()[settings_.nThreads]->setCurrentStateAndControl(x_[K_], control_vector_t::Zero(), settings_.dt*K_);

	// derivative of termination cost with respect to state
	q_[K_] = this->getCostFunctionInstances()[settings_.nThreads]->evaluateTerminal();
	qv_[K_] = this->getCostFunctionInstances()[settings_.nThreads]->stateDerivativeTerminal();
	Q_[K_] = this->getCostFunctionInstances()[settings_.nThreads]->stateSecondDerivativeTerminal();

	// initialize quadratic approximation of cost to go
	S_[K_] = Q_[K_];
	sv_[K_] = qv_[K_];
}



template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::computeCostToGo(size_t k)
{
	S_[k] = Q_[k];
	S_[k].noalias() += A_[k].transpose() * S_[k+1] * A_[k];
	S_[k].noalias() -= L_[k].transpose() * Hi_[k] * L_[k];

	S_[k] = 0.5*(S_[k]+S_[k].transpose()).eval();

	sv_[k] = qv_[k];
	sv_[k].noalias() += A_[k].transpose() * sv_[k+1];
	sv_[k].noalias() += L_[k].transpose() * Hi_[k] * lv_[k];
	sv_[k].noalias() += L_[k].transpose() * gv_[k];
	sv_[k].noalias() += G_[k].transpose() * lv_[k];

}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::designController(size_t k)
{
	gv_[k] = rv_[k];
	gv_[k].noalias() += B_[k].transpose() * sv_[k+1];

	G_[k] = P_[k];
	//G_[k].noalias() += B_[k].transpose() * S_[k+1] * A_[k];
	G_[k].noalias() += B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * A_[k];

	H_[k] = R_[k];
	//H_[k].noalias() += B_[k].transpose() * S_[k+1] * B_[k];
	H_[k].noalias() += B_[k].transpose() * S_[k+1].template selfadjointView<Eigen::Lower>() * B_[k];

	if(settings_.fixedHessianCorrection)
	{
		if (settings_.epsilon > 1e-10)
			Hi_[k] = H_[k] + settings_.epsilon*control_matrix_t::Identity();
		else
			Hi_[k] = H_[k];

		if (settings_.recordSmallestEigenvalue)
		{
			// compute eigenvalues with eigenvectors enabled
			eigenvalueSolver_.compute(Hi_[k], Eigen::ComputeEigenvectors);
			const control_matrix_t& V = eigenvalueSolver_.eigenvectors().real();
			const control_vector_t& lambda = eigenvalueSolver_.eigenvalues();

			smallestEigenvalue_ = std::min(smallestEigenvalue_, lambda.minCoeff());
			smallestEigenvalueIteration_ = std::min(smallestEigenvalueIteration_, lambda.minCoeff());

			// Corrected Eigenvalue Matrix
			control_matrix_t D = control_matrix_t::Zero();
			// make D positive semi-definite (as described in IV. B.)
			D.diagonal() = lambda.cwiseMax(settings_.epsilon);

			// reconstruct H
			control_matrix_t Hi_regular = V * D * V.transpose();

			// invert D
			control_matrix_t D_inverse = control_matrix_t::Zero();
			// eigenvalue-wise inversion
			D_inverse.diagonal() = -1.0 * D.diagonal().cwiseInverse();
			control_matrix_t Hi_inverse_regular = V * D_inverse * V.transpose();

			if (!Hi_inverse_[k].isApprox(Hi_inverse_regular, 1e-4))
			{
				std::cout << "warning, inverses not identical at "<<k<<std::endl;
				std::cout << "Hi_inverse_fixed - Hi_inverse_regular: "<<std::endl<<Hi_inverse_[k]-Hi_inverse_regular<<std::endl<<std::endl;
			}

		}

		Hi_inverse_[k] = -Hi_[k].template selfadjointView<Eigen::Lower>().llt().solve(control_matrix_t::Identity());

		// calculate FB gain update
		L_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * G_[k];

		// calculate FF update
		lv_[k].noalias() = Hi_inverse_[k].template selfadjointView<Eigen::Lower>() * gv_[k];


	} else {

		// compute eigenvalues with eigenvectors enabled
		eigenvalueSolver_.compute(H_[k], Eigen::ComputeEigenvectors);
		const control_matrix_t& V = eigenvalueSolver_.eigenvectors().real();
		const control_vector_t& lambda = eigenvalueSolver_.eigenvalues();

		if (settings_.recordSmallestEigenvalue)
		{
			smallestEigenvalue_ = std::min(smallestEigenvalue_, lambda.minCoeff());
			smallestEigenvalueIteration_ = std::min(smallestEigenvalueIteration_, lambda.minCoeff());
		}

		// Corrected Eigenvalue Matrix
		control_matrix_t D = control_matrix_t::Zero();
		// make D positive semi-definite (as described in IV. B.)
		D.diagonal() = lambda.cwiseMax(settings_.epsilon);

		// reconstruct H
		Hi_[k].noalias() = V * D * V.transpose();

		// invert D
		control_matrix_t D_inverse = control_matrix_t::Zero();
		// eigenvalue-wise inversion
		D_inverse.diagonal() = -1.0 * D.diagonal().cwiseInverse();
		Hi_inverse_[k].noalias() = V * D_inverse * V.transpose();

		// calculate FB gain update
		L_[k].noalias() = Hi_inverse_[k] * G_[k];

		// calculate FF update
		lv_[k].noalias() = Hi_inverse_[k] * gv_[k];
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::debugPrint()
{
	std::cout<<"iteration "  << iteration_ << std::endl;
	std::cout<<"============"<< std::endl;

	std::cout<<std::setprecision(15) << "intermediate cost: " << intermediateCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "final cost:        " << finalCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "total cost:        " << intermediateCostBest_ + finalCostBest_ << std::endl;

	if(settings_.recordSmallestEigenvalue)
	{
		std::cout<<std::setprecision(15) << "smallest eigenvalue this iteration: " << smallestEigenvalueIteration_ << std::endl;
		std::cout<<std::setprecision(15) << "smallest eigenvalue overall:        " << smallestEigenvalue_ << std::endl;
	}

	std::cout<<"                   ========" << std::endl;
	std::cout<<std::endl;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::logToMatlab()
{
	// all the variables in MATLAB that are ended by "_"
	// will be saved in a mat-file

#ifdef MATLAB
	matFile_.open("iLQGLog"+std::to_string(iteration_)+".mat");

	matFile_.put("iteration", iteration_);
	matFile_.put("K", K_);
	matFile_.put("x", x_.toImplementation());
	matFile_.put("u", u_.toImplementation());
	matFile_.put("A", A_.toImplementation());
	matFile_.put("B", B_.toImplementation());
	matFile_.put("qv", qv_.toImplementation());
	matFile_.put("Q", Q_.toImplementation());
	matFile_.put("P", P_.toImplementation());
	matFile_.put("rv", rv_.toImplementation());
	matFile_.put("R", R_.toImplementation());
	matFile_.put("sv", sv_.toImplementation());
	matFile_.put("S", S_.toImplementation());
	matFile_.put("L", L_.toImplementation());
	matFile_.put("lv", lv_.toImplementation());
	matFile_.put("u_ff", u_ff_.toImplementation());
	matFile_.put("H", H_.toImplementation());
	matFile_.put("Hi_", Hi_.toImplementation());
	matFile_.put("Hi_inverse", Hi_inverse_.toImplementation());
	matFile_.put("G", G_.toImplementation());
	matFile_.put("gv", gv_.toImplementation());
	matFile_.put("q", q_);

	matFile_.close();
#endif //MATLAB
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const core::ControlTrajectory<CONTROL_DIM, SCALAR> iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::getControlTrajectory() const
{
	// TODO this method currently copies the time array (suboptimal)

	core::tpl::TimeArray<SCALAR> t_control = t_;
	t_control.pop_back();

	return core::ControlTrajectory<CONTROL_DIM, SCALAR> (t_control, u_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR iLQGBase<STATE_DIM, CONTROL_DIM, SCALAR>::getCost() const
{
	return lowestCost_;
}


}
}
