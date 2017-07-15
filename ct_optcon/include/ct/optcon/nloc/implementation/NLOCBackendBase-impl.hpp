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


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setInitialGuess(const Policy_t& initialGuess)
{
	if(initialGuess.uff().size() != initialGuess.x_ref().size()-1)
	{
		std::cout << "Provided initial state and control trajectories are not of correct size. Control should be one shorter than state.";
		std::cout << "Control length is "<<initialGuess.uff().size()<<" but state length is "<<initialGuess.x_ref().size()<<std::endl;
		throw(std::runtime_error("state and control trajectories are not equally long"));
	}

	if(initialGuess.uff().size() < K_){
		std::cout << "Initial guess length too short. Received length " << initialGuess.uff().size() <<", expected " << K_ << std::endl;
		throw std::runtime_error("initial control guess to short");
	}

	if(initialGuess.uff().size() > K_)
		std::cout << "Warning, initial control guess too long, will truncate" << std::endl;

	u_ff_ = initialGuess.uff();
	x_ = initialGuess.x_ref();

	initialized_ = true;

	t_ = TimeArray(settings_.dt_sim, x_.size(), 0.0);

	reset();

	// compute costs of the initial guess trajectory
	computeCostsOfTrajectory(settings_.nThreads, x_, u_ff_, intermediateCostBest_, finalCostBest_);
	intermediateCostPrevious_ = intermediateCostBest_;
	finalCostPrevious_ = finalCostBest_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeTimeHorizon(const SCALAR& tf)
{
	if (tf < 0)
		throw std::runtime_error("negative time horizon specified");

	int K = settings_.computeK(tf);

	if (K == K_) return;

	K_ = K;

	x_.resize(K_+1);
	xShot_.resize(K_+1);
	u_ff_.resize(K_);
	u_ff_prev_.resize(K_);
	L_.resize(K);

	lqocProblem_->changeNumStages(K_);
	lqocSolver_->setProblem(lqocProblem_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0)
{
	if (x_.size() == 0)
		x_.resize(1);

	x_[0] = x0;
	reset(); // since initial state changed, we have to start fresh, i.e. with a rollout
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf)
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

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn)
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
		systems_[i] = typename Base::OptConProblem_t::DynamicsPtr_t(dyn->clone());
		systems_[i]->setController(controller_[i]);

		if(controller_[i] == nullptr)
			throw std::runtime_error("Controller not defined");

		integratorsRK4_[i] = std::shared_ptr<ct::core::IntegratorRK4<STATE_DIM, SCALAR> > (new ct::core::IntegratorRK4<STATE_DIM, SCALAR>(this->getNonlinearSystemsInstances()[i]));
		integratorsEuler_[i] = std::shared_ptr<ct::core::IntegratorEuler<STATE_DIM, SCALAR> >(new ct::core::IntegratorEuler<STATE_DIM, SCALAR>(this->getNonlinearSystemsInstances()[i]));

// @ todo: need to find different solution for that
//		if(this->getNonlinearSystemsInstances()[i]->isSymplectic())
//		{
//			integratorsEulerSymplectic_[i] = std::shared_ptr<ct::core::IntegratorSymplecticEuler<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>>(
//									new ct::core::IntegratorSymplecticEuler<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>(
//										std::static_pointer_cast<ct::core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>> (this->getNonlinearSystemsInstances()[i])));
//			integratorsRkSymplectic_[i] = std::shared_ptr<ct::core::IntegratorSymplecticRk<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>>(
//									new ct::core::IntegratorSymplecticRk<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>(
//										std::static_pointer_cast<ct::core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR>> (this->getNonlinearSystemsInstances()[i])));
//		}
	}
	reset(); // since system changed, we have to start fresh, i.e. with a rollout
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin)
{
//	linearSystemDiscretizers_.resize(settings_.nThreads+1);

	getLinearSystemsInstances().resize(settings_.nThreads+1);

	for (size_t i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		getLinearSystemsInstances()[i] = typename OptConProblem_t::LinearPtr_t(lin->clone());

		linearSystemDiscretizers_[i].setLinearSystem(getLinearSystemsInstances()[i]);
	}
	// technically a linear system change does not require a new rollout. Hence, we do not reset.
}




template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::checkProblem()
{
	if (K_==0)
		throw std::runtime_error("Time horizon too small resulting in 0 GNMS steps");

	if (u_ff_.size() < K_)
	{
		std::cout << "Provided initial feed forward controller too short, should be at least "<<K_<<" but is " << u_ff_.size() <<" long."<<std::endl;
		throw(std::runtime_error("Provided initial feed forward controller too short"));
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::configure(
	const Settings_t& settings)
{
	if (!settings.parametersOk())
	{
		throw(std::runtime_error("GNMSSettings are incorrect. Aborting."));
	}

	if (settings.nThreads != settings_.nThreads)
	{
		throw(std::runtime_error("Number of threads at GNMS cannot be changed after instance has been created."));
	}

	// update system discretizer with new settings
	for(size_t i = 0; i< settings.nThreads+1; i++)
	{
		linearSystemDiscretizers_[i].setApproximation(settings.discretization);
		linearSystemDiscretizers_[i].setTimeDiscretization(settings.dt);
	}


	// select the linear quadratic solver based on settings file
	if(settings.lqocp_solver == NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER)
	{
		lqocSolver_ = std::shared_ptr<GNRiccatiSolver<STATE_DIM, CONTROL_DIM, SCALAR>>(
				new GNRiccatiSolver<STATE_DIM, CONTROL_DIM, SCALAR>());
	}
	else if (settings.lqocp_solver == NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER)
	{
#ifdef HPIPM
		lqocSolver_ = std::shared_ptr<HPIPMInterface<STATE_DIM, CONTROL_DIM>>(
				new HPIPMInterface<STATE_DIM, CONTROL_DIM>());
#else
		throw std::runtime_error("HPIPM selected but not built.");
#endif
	}
	else
		throw std::runtime_error("Solver for Linear Quadratic Optimal Control Problem wrongly specified.");


	// will be set correctly later
	Eigen::setNbThreads(settings.nThreadsEigen);

	lqocSolver_->configure(settings);

	settings_ = settings;

	reset();

	configured_ = true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::rolloutSystem(
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

	//it's a bit ugly to have this as a temporary variable here, but it avoids making another reference xtrajectory member
	// todo: maybe find cleaner solution
	ct::core::StateVectorArray<STATE_DIM, SCALAR> x_ref = x_;

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

		u_local.push_back( u_ff_local[i] + L_[i] * (x0-x_ref[i]));
		controller_[threadId]->setControl(u_local.back());

		for (size_t j=0; j<steps; j++)
		{
			if (steps > 1)
			{
				controller_[threadId]->setControl(u_local.back() + L_[i]*(x0-x_ref[i]));
			}

			if (settings_.integrator == GNMSSettings::EULER)
			{
				integratorsEuler_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
			}
			else if(settings_.integrator == GNMSSettings::RK4)
			{
				integratorsRK4_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
			}
			// todo: find cleaner solution for symplectic stuff
//			else if(settings_.integrator == GNMSSettings::EULER_SYM)
//			{
//				integratorsEulerSymplectic_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
//			}
//			else if(settings_.integrator == GNMSSettings::RK_SYM)
//			{
//				integratorsRkSymplectic_[threadId]->integrate_n_steps(x0, (i*steps+j)*dt_sim, 1, dt_sim);
//			}
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



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::sequentialLQProblem()
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

	start = std::chrono::steady_clock::now();
	if (iteration_ == 0)
		initializeShots();
	else
		updateShots();
	end = std::chrono::steady_clock::now();
	diff = end - start;
#ifdef DEBUG_PRINT
	std::cout << "Shot integration took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif

	start = std::chrono::steady_clock::now();
	computeDefects();
	end = std::chrono::steady_clock::now();
	diff = end - start;
#ifdef DEBUG_PRINT
	std::cout << "Defects computation took "<<std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
#endif
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeSingleShot(size_t threadId, size_t k)
{
	const double& dt_sim = settings_.dt_sim;

	controller_[threadId]->setControl(u_ff_[k]);
	xShot_[k] = x_[k];

	if (settings_.integrator == GNMSSettings::EULER)
	{
		integratorsEuler_[threadId]->integrate_n_steps(xShot_[k], k*dt_sim, 1, dt_sim);
	}
	else if(settings_.integrator == GNMSSettings::RK4)
	{
		integratorsRK4_[threadId]->integrate_n_steps(xShot_[k], k*dt_sim, 1, dt_sim);
	}
	//! todo need to find different solution for that
//	else if(settings_.integrator == GNMSSettings::EULER_SYM)
//	{
//		integratorsEulerSymplectic_[threadId]->integrate_n_steps(xShot_[k], k*dt_sim, 1, dt_sim);
//	}
//	else if(settings_.integrator == GNMSSettings::RK_SYM)
//	{
//		integratorsRkSymplectic_[threadId]->integrate_n_steps(xShot_[k], k*dt_sim, 1, dt_sim);
//	}
	else
		throw std::runtime_error("invalid integration mode selected.");
}

//template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
//void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateSingleControlAndState(size_t threadId, size_t k)
//{
//	x_[k] += lx_[k];
//	u_ff_[k] += lv_[k] + L_[k] * lx_[k];
//}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateSingleShot(size_t threadId, size_t k)
{
	// Todo: This linear update only works if lx_ and lu_ are small. Otherwise this makes it unstable.
	//xShot_[k] += (A_[k] + B_[k] * L_[k]) * lx_[k] + B_[k] * lv_[k];
	initializeSingleShot(this->settings_.nThreads, k);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeSingleDefect(size_t threadId, size_t k)
{
	if (k<K_)
		lqocProblem_->b_[k] = xShot_[k] - x_[k+1];
	else
	{
		assert(k==K_ && "k should be K_");
		lqocProblem_->b_[K_].setZero();
	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeCostsOfTrajectory(
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


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeLinearizedDynamics(size_t threadId, size_t k)
{
	LQOCProblem_t& p = *lqocProblem_;

	switch(settings_.discretization)
	{
		case core::LinearSystemDiscretizerSettings::APPROXIMATION::FORWARD_EULER:
		{
			p.A_[k] = state_matrix_t::Identity();
			p.A_[k] += settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k], u_ff_[k], k*settings_.dt);
			p.B_[k] = settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k], u_ff_[k], k*settings_.dt);
			break;
		}
		case core::LinearSystemDiscretizerSettings::APPROXIMATION::BACKWARD_EULER:
		{
			//! @todo: backward Euler uses the upcoming time-steps for computing A and B, which requires a hack for the last control input stage
			size_t k_u = std::min((size_t)K_-1, (size_t)k+1);

			state_matrix_t aNew = settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k+1], u_ff_[k_u], (k+1)*settings_.dt);
			state_matrix_t aNewInv = (state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
			p.A_[k] = aNewInv;
			p.B_[k] = aNewInv * settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k+1], u_ff_[k_u], (k+1)*settings_.dt);
			break;
		}
		case core::LinearSystemDiscretizerSettings::APPROXIMATION::TUSTIN:
		{
			//! @todo: I attempted here to implement the tustin rule correctly. Note that it is inefficient, since both ends of the time interval get evaluated.
			//! @todo most likely still wrong

			size_t k_u = std::min((size_t)K_-1, (size_t)k+1); // todo same problem as above persists here.

			state_matrix_t aNew_front = settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k], u_ff_[k], k*settings_.dt);
			state_matrix_t aNew_back = settings_.dt * this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k+1], u_ff_[k_u], k*settings_.dt);
			state_matrix_t aNew = 0.5*(aNew_front + aNew_back);

			state_control_matrix_t bNew_front = this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k], u_ff_[k], k*settings_.dt);
			state_control_matrix_t bNew_back = this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k+1], u_ff_[k_u], k*settings_.dt);
			state_control_matrix_t bNew = 0.5*(bNew_front + bNew_back);

			state_matrix_t aNewInv = (state_matrix_t::Identity() -  aNew).colPivHouseholderQr().inverse();
			p.A_[k] = aNewInv * (state_matrix_t::Identity() + aNew);
			p.B_[k] = aNewInv * settings_.dt * bNew;
			break;
		}
		case core::LinearSystemDiscretizerSettings::APPROXIMATION::MATRIX_EXPONENTIAL:
		{
			state_matrix_t Ac = this->getLinearSystemsInstances()[threadId]->getDerivativeState(x_[k], u_ff_[k], k*settings_.dt);
			state_matrix_t Adt = settings_.dt * Ac;

			p.A_[k] = Adt.exp();
			p.B_[k] = Ac.inverse() * (p.A_[k] - state_matrix_t::Identity()) *  this->getLinearSystemsInstances()[threadId]->getDerivativeControl(x_[k], u_ff_[k], k*settings_.dt);
			break;
		}

		default:
		{
			throw std::runtime_error("Unknown discretization scheme");
			break;
		}
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeQuadraticCosts(size_t threadId, size_t k)
{
	LQOCProblem_t& p = *lqocProblem_;
	const scalar_t& dt = settings_.dt;

	// feed current state and control to cost function
	this->getCostFunctionInstances()[threadId]->setCurrentStateAndControl(x_[k], u_ff_[k], dt*k);

	// derivative of cost with respect to state
	p.q_[k] = this->getCostFunctionInstances()[threadId]->evaluateIntermediate()*dt;

	p.qv_[k] = this->getCostFunctionInstances()[threadId]->stateDerivativeIntermediate()*dt;

	p.Q_[k] = this->getCostFunctionInstances()[threadId]->stateSecondDerivativeIntermediate()*dt;

	// derivative of cost with respect to control and state
	p.P_[k] = this->getCostFunctionInstances()[threadId]->stateControlDerivativeIntermediate()*dt;

	// derivative of cost with respect to control
	p.rv_[k] = this->getCostFunctionInstances()[threadId]->controlDerivativeIntermediate()*dt;

	p.R_[k] = this->getCostFunctionInstances()[threadId]->controlSecondDerivativeIntermediate()*dt;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeCostToGo()
{
	LQOCProblem_t& p = *lqocProblem_;

	// feed current state and control to cost function
	this->getCostFunctionInstances()[settings_.nThreads]->setCurrentStateAndControl(x_[K_], control_vector_t::Zero(), settings_.dt*K_);

	// derivative of termination cost with respect to state
	p.q_[K_] = this->getCostFunctionInstances()[settings_.nThreads]->evaluateTerminal();
	p.qv_[K_] = this->getCostFunctionInstances()[settings_.nThreads]->stateDerivativeTerminal();
	p.Q_[K_] = this->getCostFunctionInstances()[settings_.nThreads]->stateSecondDerivativeTerminal();
}



//template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
//void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::designStateUpdate(size_t k)
//{
//	lx_[k+1] = (A_[k] + B_[k] * L_[k]) * lx_[k]  + B_[k] * lv_[k] + lqocProblem_->b_[k];
//}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::debugPrint()
{
	std::cout<<"iteration "  << iteration_ << std::endl;
	std::cout<<"============"<< std::endl;

	std::cout<<std::setprecision(15) << "intermediate cost:         " << intermediateCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "final cost:                " << finalCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "total cost:                " << intermediateCostBest_ + finalCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "total constraint err.norm: " << d_norm_ << std::endl;
	std::cout<<std::setprecision(15) << "total state update norm:   " << dx_norm_ << std::endl;
	std::cout<<std::setprecision(15) << "total control update.norm: " << du_norm_ << std::endl;

	// todo bring back this
//	if(settings_.recordSmallestEigenvalue)
//	{
//		std::cout<<std::setprecision(15) << "smallest eigenvalue this iteration: " << smallestEigenvalueIteration_ << std::endl;
//		std::cout<<std::setprecision(15) << "smallest eigenvalue overall:        " << smallestEigenvalue_ << std::endl;
//	}

	std::cout<<"                   ========" << std::endl;
	std::cout<<std::endl;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logToMatlab(const size_t& iteration)
{
	// all the variables in MATLAB that are ended by "_"
	// will be saved in a mat-file

#ifdef MATLAB

	LQOCProblem_t& p = *lqocProblem_;

	matFile_.open("GNMSLog"+std::to_string(iteration)+".mat");

	matFile_.put("iteration", iteration);
	matFile_.put("K", K_);
	matFile_.put("x", x_.toImplementation());
	matFile_.put("u_ff", u_ff_.toImplementation());
	matFile_.put("t", t_.toEigenTrajectory());
	matFile_.put("d", lqocProblem_->b_.toImplementation());
	matFile_.put("xShot", xShot_.toImplementation());

	matFile_.put("A", p.A_.toImplementation());
	matFile_.put("B", p.B_.toImplementation());
	matFile_.put("qv", p.qv_.toImplementation());
	matFile_.put("Q", p.Q_.toImplementation());
	matFile_.put("P", p.P_.toImplementation());
	matFile_.put("rv", p.rv_.toImplementation());
	matFile_.put("R", p.R_.toImplementation());
	matFile_.put("q", p.q_.toEigenTrajectory());

//	matFile_.put("sv", sv_.toImplementation());
//	matFile_.put("S", S_.toImplementation());
//	matFile_.put("L", L_.toImplementation());
//	matFile_.put("lv", lv_.toImplementation());
//	matFile_.put("lx", lx_.toImplementation());
//	matFile_.put("H", H_.toImplementation());
//	matFile_.put("Hi_", Hi_.toImplementation());
//	matFile_.put("Hi_inverse", Hi_inverse_.toImplementation());
//	matFile_.put("G", G_.toImplementation());
//	matFile_.put("gv", gv_.toImplementation());


	matFile_.close();
#endif //MATLAB
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logInitToMatlab()
{
	// all the variables in MATLAB that are ended by "_"
	// will be saved in a mat-file

#ifdef MATLAB

	matFile_.open("GNMSLogInit.mat");

	matFile_.put("xInit", x_.toImplementation());
	matFile_.put("u_ffInit", u_ff_.toImplementation());

	matFile_.close();
#endif //MATLAB
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::ControlTrajectory<CONTROL_DIM, SCALAR> NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getControlTrajectory() const
{
	// TODO this method currently copies the time array (suboptimal)

	core::tpl::TimeArray<SCALAR> t_control = t_;
	t_control.pop_back();

	return core::ControlTrajectory<CONTROL_DIM, SCALAR> (t_control, u_ff_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::StateTrajectory<STATE_DIM, SCALAR> NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getStateTrajectory() const
{
	// TODO this method currently copies the time array (suboptimal)

	core::tpl::TimeArray<SCALAR> t_control = t_;

	return core::StateTrajectory<STATE_DIM, SCALAR> (t_control, x_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getCost() const
{
	return intermediateCostBest_ + finalCostBest_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::lineSearchController()
{
	// if first iteration, we have to find cost of initial rollout
	if (iteration_ == 0)
	{
		intermediateCostBest_ = 0.0;

		for (int k=K_-1; k>=0; k--)
			intermediateCostBest_ += lqocProblem_->q_[k];

		finalCostBest_ = lqocProblem_->q_[K_];
	}

	// lowest cost is cost of last rollout
	lowestCost_ = intermediateCostBest_ + finalCostBest_;
	scalar_t lowestCostPrevious = lowestCost_;

	if (!settings_.lineSearchSettings.active)
	{
		ControlVectorArray u_recorded(K_);
		TimeArray t_local(K_);

		bool dynamicsGood = rolloutSystem(settings_.nThreads, u_ff_, x_, u_recorded, t_local);

		if (dynamicsGood)
		{
			intermediateCostBest_ = std::numeric_limits<scalar_t>::max();
			finalCostBest_ = std::numeric_limits<scalar_t>::max();
			computeCostsOfTrajectory(settings_.nThreads, x_, u_recorded, intermediateCostBest_, finalCostBest_);
			lowestCost_ = intermediateCostBest_ + finalCostBest_;
			u_ff_.swap(u_recorded);
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

}
}
