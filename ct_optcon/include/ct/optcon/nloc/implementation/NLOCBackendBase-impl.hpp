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

	if(initialGuess.uff().size() < (size_t)K_){
		std::cout << "Initial guess length too short. Received length " << initialGuess.uff().size() <<", expected " << K_ << std::endl;
		throw std::runtime_error("initial control guess to short");
	}

	if(initialGuess.uff().size() > (size_t)K_)
		std::cout << "Warning, initial control guess too long, will truncate" << std::endl;

	if(initialGuess.K().size() < (size_t)K_){
		std::cout << "Initial feedback length too short. Received length " << initialGuess.K().size() <<", expected " << K_ << std::endl;
		throw std::runtime_error("initial control guess to short");
	}

	if(initialGuess.K().size() > (size_t)K_)
		std::cout << "Warning, initial feedback guess too long, will truncate" << std::endl;


	u_ff_ = initialGuess.uff();
	L_ = initialGuess.K();
	x_ = initialGuess.x_ref();
	x_prev_ = x_;

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

	K_ = K;

	lx_.resize(K_+1);
	x_.resize(K_+1);
	x_prev_.resize(K_+1);
	xShot_.resize(K_+1);

	lv_.resize(K_);
	lu_.resize(K_);
	u_ff_.resize(K_);
	u_ff_prev_.resize(K_);
	L_.resize(K);

	lqocProblem_->changeNumStages(K_);
	lqocProblem_->setZero();

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

	costFunctions_.resize(settings_.nThreads+1);

	for (int i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		costFunctions_[i] = typename Base::OptConProblem_t::CostFunctionPtr_t(cf->clone());
	}

	// recompute cost if line search is active
	if (iteration_ > 0 && settings_.lineSearchSettings.active)
		computeQuadraticCostsAroundTrajectory(0, K_-1);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn)
{
	if (dyn == nullptr)
		throw std::runtime_error("system dynamics are nullptr");

	systems_.resize(settings_.nThreads+1);
	integrators_.resize(settings_.nThreads+1);
	integratorsEulerSymplectic_.resize(settings_.nThreads+1);
	integratorsRkSymplectic_.resize(settings_.nThreads+1);

	for (int i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		systems_[i] = typename Base::OptConProblem_t::DynamicsPtr_t(dyn->clone());
		systems_[i]->setController(controller_[i]);

		if(controller_[i] == nullptr)
			throw std::runtime_error("Controller not defined");

		integrators_[i] = std::shared_ptr<ct::core::Integrator<STATE_DIM, SCALAR> >(new ct::core::Integrator<STATE_DIM, SCALAR>(systems_[i], settings_.integrator));

		initializeSymplecticIntegrators<V_DIM, P_DIM>(i);
	}
	reset(); // since system changed, we have to start fresh, i.e. with a rollout
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_ENABLED
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeSymplecticIntegrators(size_t i)
{
	if(systems_[i]->isSymplectic())
	{
		//! it only makes sense to compile the following code, if V_DIM > 0 and P_DIM > 0
		integratorsEulerSymplectic_[i] = std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
								new ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
									std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>> (systems_[i])));
		integratorsRkSymplectic_[i] = std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
								new ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
									std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>> (systems_[i])));
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin)
{
	linearSystems_.resize(settings_.nThreads+1);

	for (int i = 0; i<settings_.nThreads+1; i++)
	{
		// make a deep copy
		linearSystems_[i] = typename OptConProblem_t::LinearPtr_t(lin->clone());

		linearSystemDiscretizers_[i].setLinearSystem(linearSystems_[i]);
	}
	// technically a linear system change does not require a new rollout. Hence, we do not reset.
}




template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::checkProblem()
{
	if (K_==0)
		throw std::runtime_error("Time horizon too small resulting in 0 steps");

	if (u_ff_.size() < (size_t)K_)
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
		throw(std::runtime_error("Settings are incorrect. Aborting."));
	}

	if (settings.nThreads != settings_.nThreads)
	{
		throw(std::runtime_error("Number of threads cannot be changed after instance has been created."));
	}

	// update system discretizer with new settings
	for(int i = 0; i< settings.nThreads+1; i++)
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

	// set number of Eigen Threads (requires -fopenmp)
	if (settings_.nThreadsEigen > 1)
	{
		Eigen::setNbThreads(settings.nThreadsEigen);
#ifdef DEBUG_PRINT_MP
	std::cout << "[MP] Eigen using " << Eigen::nbThreads() << " threads." << std::endl;
#endif
	}

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

		if(settings_.closedLoopShooting  || firstRollout_)
			u_local.push_back(u_ff_local[i] + L_[i] * (x0 - x_prev_[i]));
		else
			u_local.push_back(u_ff_local[i]);

		controller_[threadId]->setControl(u_local.back());

		if(settings_.integrator == ct::core::IntegrationType::EULER_SYM  || settings_.integrator == ct::core::IntegrationType::RK_SYM)
		{
			integrateSymplectic<V_DIM, P_DIM>(threadId, x0, i*dt, steps, dt_sim);
		}
		else
		{
			integrators_[threadId]->integrate_n_steps(x0, i*dt, steps, dt_sim);
		}

		x_local.push_back(x0);
		t_local.push_back((i+1)*dt);

		// check if nan
		for (size_t k=0; k<STATE_DIM; k++)
		{
			if (isnan(x_local[i](k)))
			{
				x_local.resize(K_local+1, ct::core::StateVector<STATE_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				u_local.resize(K_local, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				t_local.resize(K_local+1, std::numeric_limits<SCALAR>::quiet_NaN());
				return false;
			}
		}
		for (size_t k=0; k<CONTROL_DIM; k++)
		{
			if (isnan(u_local[i](k)))
			{
				x_local.resize(K_local+1, ct::core::StateVector<STATE_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				u_local.resize(K_local, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				t_local.resize(K_local+1, std::numeric_limits<SCALAR>::quiet_NaN());
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
SYMPLECTIC_ENABLED
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::integrateSymplectic(size_t threadId, ct::core::StateVector<STATE_DIM, SCALAR>& x0, const double& t, const size_t& steps, const double& dt_sim) const
{
	if (!systems_[threadId]->isSymplectic())
		throw std::runtime_error("Trying to integrate using symplectic integrator, but system is not symplectic.");

	if(settings_.integrator == ct::core::IntegrationType::EULER_SYM)
	{
		integratorsEulerSymplectic_[threadId]->integrate_n_steps(x0, t, steps, dt_sim);
	}
	else if(settings_.integrator == ct::core::IntegrationType::RK_SYM)
	{
		integratorsRkSymplectic_[threadId]->integrate_n_steps(x0, t, steps, dt_sim);
	} else
	{
		throw std::runtime_error("invalid symplectic integrator specified");
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::rolloutShotsSingleThreaded(size_t threadId, size_t firstIndex, size_t lastIndex,
		const ControlVectorArray& u_ff_local, const StateVectorArray& x_start, StateVectorArray& xShot, StateVectorArray& d) const
{
	//! make sure all intermediate entries in the defect trajectory are zero
	d.setConstant(state_vector_t::Zero());

	for (size_t k=firstIndex; k<=lastIndex; k++)
	{
		// first rollout the shot
		rolloutSingleShot(threadId, k, u_ff_local[k], x_start[k], x_prev_[k], L_[k], xShot[k]);

		// then compute the corresponding defect
		computeSingleDefect(k, x_start[k], xShot[k+1], d[k]);
	}
}



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::rolloutSingleShot(
		const size_t threadId,
		const size_t k,
		const control_vector_t& u_ff_local,
		const state_vector_t& x_start,
		const state_vector_t& x_prev,
		const feedback_matrix_t& L,
		state_vector_t& xShot) const
{
	const double& dt = settings_.dt;
	const double& dt_sim = settings_.dt_sim;

	// compute number of substeps
	size_t steps = round(dt/ dt_sim);

	if(settings_.closedLoopShooting)
		controller_[threadId]->setControl(u_ff_local + L * (x_start - x_prev));
	else
		controller_[threadId]->setControl(u_ff_local);

	xShot = x_start;

	if(settings_.integrator == ct::core::IntegrationType::EULER_SYM || settings_.integrator == ct::core::IntegrationType::RK_SYM)
	{
		integrateSymplectic<V_DIM, P_DIM>(threadId, xShot, k*dt_sim, 1, dt_sim);
	} else
	{
		integrators_[threadId]->integrate_n_steps(xShot, k*dt, steps, dt_sim);
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeSingleDefect(size_t k,
		const state_vector_t& x_start, const state_vector_t& xShot, state_vector_t& d) const
{
	if (k< (size_t)K_)
	{
		d = xShot - x_start;
	}
	else
	{
		d.setZero();
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

	for (size_t k=0; k<(size_t)K_; k++)
	{
		// feed current state and control to cost function
		costFunctions_[threadId]->setCurrentStateAndControl(x_local[k], u_local[k], settings_.dt*k);

		// derivative of cost with respect to state
		intermediateCost += costFunctions_[threadId]->evaluateIntermediate();
	}
	intermediateCost *= settings_.dt;

	costFunctions_[threadId]->setCurrentStateAndControl(x_local[K_], control_vector_t::Zero(), settings_.dt*K_);
	finalCost = costFunctions_[threadId]->evaluateTerminal();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeLinearizedDynamics(size_t threadId, size_t k)
{
	LQOCProblem_t& p = *lqocProblem_;

	/*!
	 * @todo
	 * Note the little 'hack' that is applied here. We need a control input to linearize around for the last stage N.
	 * Should it be zero? We currently set it to be the second-to-last control input.
	 */
	const core::ControlVector<CONTROL_DIM, SCALAR> u_last = u_ff_[std::min((int)k+1, K_-1)];

	assert(lqocProblem_ != nullptr);

	assert(&lqocProblem_->A_[k] != nullptr);
	assert(&lqocProblem_->B_[k] != nullptr);

	assert(lqocProblem_->A_.size() > k);
	assert(lqocProblem_->B_.size() > k);

	if (settings_.timeVaryingDiscretization)
		linearSystemDiscretizers_[threadId].getAandBTimeVarying(x_[k], u_ff_[k], x_[k+1], u_last, (int)k, p.A_[k], p.B_[k]);
	else
		linearSystemDiscretizers_[threadId].getAandB(x_[k], u_ff_[k], (int)k, p.A_[k], p.B_[k]);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeQuadraticCosts(size_t threadId, size_t k)
{
	LQOCProblem_t& p = *lqocProblem_;
	const scalar_t& dt = settings_.dt;

	// feed current state and control to cost function
	costFunctions_[threadId]->setCurrentStateAndControl(x_[k], u_ff_[k], dt*k);

	// derivative of cost with respect to state
	p.q_[k] = costFunctions_[threadId]->evaluateIntermediate()*dt;

	p.qv_[k] = costFunctions_[threadId]->stateDerivativeIntermediate()*dt;

	p.Q_[k] = costFunctions_[threadId]->stateSecondDerivativeIntermediate()*dt;

	// derivative of cost with respect to control and state
	p.P_[k] = costFunctions_[threadId]->stateControlDerivativeIntermediate()*dt;

	// derivative of cost with respect to control
	p.rv_[k] = costFunctions_[threadId]->controlDerivativeIntermediate()*dt;

	p.R_[k] = costFunctions_[threadId]->controlSecondDerivativeIntermediate()*dt;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeCostToGo()
{
	LQOCProblem_t& p = *lqocProblem_;

	// feed current state and control to cost function
	costFunctions_[settings_.nThreads]->setCurrentStateAndControl(x_[K_], control_vector_t::Zero(), settings_.dt*K_);

	// derivative of termination cost with respect to state
	p.q_[K_] = costFunctions_[settings_.nThreads]->evaluateTerminal();
	p.qv_[K_] = costFunctions_[settings_.nThreads]->stateDerivativeTerminal();
	p.Q_[K_] = costFunctions_[settings_.nThreads]->stateSecondDerivativeTerminal();
}



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::debugPrint()
{

	SCALAR d_norm_l1 = computeDefectsNorm<1>(lqocProblem_->b_);
	SCALAR d_norm_l2 = computeDefectsNorm<2>(lqocProblem_->b_);

	std::cout<< settings_.loggingPrefix + " iteration "  << iteration_ << std::endl;
	std::cout<<"============"<< std::endl;

	std::cout<<std::setprecision(15) << "interm. cost:\t" << intermediateCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "final cost:\t" << finalCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "total cost:\t" << intermediateCostBest_ + finalCostBest_ << std::endl;
	std::cout<<std::setprecision(15) << "total merit:\t" << intermediateCostBest_ + finalCostBest_ + settings_.meritFunctionRho * d_norm_ << std::endl;
	std::cout<<std::setprecision(15) << "tot. defect L1:\t" << d_norm_l1 << std::endl;
	std::cout<<std::setprecision(15) << "tot. defect L2:\t" << d_norm_l2 << std::endl;
	std::cout<<std::setprecision(15) << "total lx norm:\t" << lx_norm_ << std::endl;
	std::cout<<std::setprecision(15) << "total lu norm:\t" << lu_norm_ << std::endl;
	std::cout<<std::setprecision(15) << "step-size(alpha):\t" << alphaBest_ << std::endl;

	if(settings_.recordSmallestEigenvalue && settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
	{
		std::cout<<std::setprecision(15) << "smallest eigenvalue this iteration: " << lqocSolver_->getSmallestEigenvalue() << std::endl;
	}

	std::cout<<"                   ========" << std::endl;
	std::cout<<std::endl;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logToMatlab(const size_t& iteration)
{
#ifdef MATLAB_FULL_LOG

	std::cout << "Logging to Matlab" << std::endl;

	LQOCProblem_t& p = *lqocProblem_;

	matFile_.open(settings_.loggingPrefix+"Log"+std::to_string(iteration)+".mat");

	matFile_.put("iteration", iteration);
	matFile_.put("K", K_);
	matFile_.put("dt", settings_.dt);
	matFile_.put("dt_sim", settings_.dt_sim);
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

	matFile_.put("lx_norm", lx_norm_);
	matFile_.put("lu_norm", lu_norm_);
	matFile_.put("cost", getCost());
	matFile_.put("alphaStep", alphaBest_);

	d_norm_ = computeDefectsNorm<1>(lqocProblem_->b_);
	matFile_.put("d_norm", d_norm_);

	matFile_.close();
#endif //MATLAB_FULL_LOG
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logInitToMatlab()
{

#ifdef MATLAB

	std::cout << "Logging init guess to Matlab" << std::endl;

	matFile_.open(settings_.loggingPrefix+"LogInit.mat");

	matFile_.put("K", K_);
	matFile_.put("dt", settings_.dt);
	matFile_.put("dt_sim", settings_.dt_sim);

	matFile_.put("x", x_.toImplementation());
	matFile_.put("u_ff", u_ff_.toImplementation());
	matFile_.put("d", lqocProblem_->b_.toImplementation());
	matFile_.put("cost", getCost());

	d_norm_ = computeDefectsNorm<1>(lqocProblem_->b_);
	matFile_.put("d_norm", d_norm_);

	matFile_.close();
#endif
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
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::lineSearchSingleShooting()
{

	// lowest cost is cost of last rollout
	lowestCost_ = intermediateCostBest_ + finalCostBest_;
	scalar_t lowestCostPrevious = lowestCost_;

	//! backup controller that led to current trajectory
	u_ff_prev_ = u_ff_;
	x_prev_ = x_;

	if (!settings_.lineSearchSettings.active)
	{
		ControlVectorArray uff_local(K_);
		ControlVectorArray u_recorded(K_);
		TimeArray t_local(K_+1);


		for(int i = 0; i< K_; i++)
		{
			if(settings_.closedLoopShooting)
			{
				uff_local[i] = u_ff_[i] + lv_[i]; 	// add lv_ if we are doing closed-loop shooting
			}
			else
				uff_local[i] = u_ff_[i] + lu_[i]; 	// add lu if we are doing open-loop shooting
		}

		bool dynamicsGood = rolloutSystem(settings_.nThreads, uff_local, x_, u_recorded, t_local);

		if (dynamicsGood)
		{
			intermediateCostBest_ = std::numeric_limits<scalar_t>::max();
			finalCostBest_ = std::numeric_limits<scalar_t>::max();
			computeCostsOfTrajectory(settings_.nThreads, x_, u_recorded, intermediateCostBest_, finalCostBest_);
			lowestCost_ = intermediateCostBest_ + finalCostBest_;

#if defined (MATLAB_FULL_LOG) || defined (DEBUG_PRINT)
			//! compute l2 norms of state and control update
			lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(u_recorded, u_ff_);
			lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(x_prev_, x_);
#endif

			x_prev_ = x_;
			u_ff_.swap(u_recorded);
			t_.swap(t_local);
			alphaBest_ = 1;
		}
		else
		{
#ifdef DEBUG_PRINT
std::cout<<"CONVERGED: System became unstable!" << std::endl;
#endif //DEBUG_PRINT
			return false;
		}
	}
	else
	{

#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Starting line search."<<std::endl;
		std::cout<<"[LineSearch]: Cost last rollout: "<<lowestCost_<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

		alphaBest_ = performLineSearch();

#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Best control found at alpha: "<<alphaBest_<<" . Will use this control."<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

#ifdef DEBUG_PRINT
		if (alphaBest_ == 0.0)
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



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::executeLineSearchSingleShooting(
		const size_t threadId,
		const scalar_t alpha,
		const ControlVectorArray& u_ff_update,
		ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_recorded,
		ct::core::tpl::TimeArray<SCALAR>& t_local,
		scalar_t& intermediateCost,
		scalar_t& finalCost,
		std::atomic_bool* terminationFlag
) const
{
	intermediateCost =  std::numeric_limits<scalar_t>::infinity();
	finalCost = std::numeric_limits<scalar_t>::infinity();

	if (terminationFlag && *terminationFlag) return;

	ControlVectorArray u_ff_alpha (K_);

	for (int k=K_-1; k>=0; k--)
	{
		u_ff_alpha[k] = alpha * u_ff_update[k] + u_ff_prev_[k];
	}

	bool dynamicsGood = rolloutSystem(threadId, u_ff_alpha, x_local, u_recorded, t_local, terminationFlag);

	if (terminationFlag && *terminationFlag) return;

	if (dynamicsGood)
	{
		computeCostsOfTrajectory(threadId, x_local, u_recorded, intermediateCost, finalCost);
	}
	else
	{
#ifdef DEBUG_PRINT
		std::string msg = std::string("dynamics not good, thread: ") + std::to_string(threadId);
		std::cout << msg << std::endl;
#endif
	}
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::lineSearchMultipleShooting()
{

	// lowest cost
	scalar_t lowestCostPrevious;

	//! backup controller that led to current trajectory
	u_ff_prev_ = u_ff_;
	x_prev_ = x_;


	if (!settings_.lineSearchSettings.active)	//! do full step updates
	{
		// lowest cost is cost of last rollout
		lowestCostPrevious = intermediateCostBest_ + finalCostBest_;

		for(int i = 0; i< K_; i++)
		{
			u_ff_[i] += lu_[i];
		}

		for(int i = 0; i<K_+1; i++)
		{
			x_[i] += lx_[i];
		}

		rolloutShots(0, K_-1);

		updateCosts();

		lowestCost_ = intermediateCostBest_ + finalCostBest_;

#if defined (MATLAB_FULL_LOG) || defined (DEBUG_PRINT)
		lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(u_ff_prev_, u_ff_);
		lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(x_prev_, x_);
#endif
		x_prev_ = x_;
		alphaBest_ = 1;
	}
	else //! do line search over a merit function trading off costs and constraint violations
	{
		// merit of previous trajectory
		lowestCost_ = intermediateCostBest_ + finalCostBest_ + d_norm_ * settings_.meritFunctionRho;
		lowestCostPrevious = lowestCost_;

#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Starting line search."<<std::endl;
		std::cout<<"[LineSearch]: Cost of last rollout:\t"<<intermediateCostBest_ + finalCostBest_<<std::endl;
		std::cout<<"[LineSearch]: Defect norm last rollout:\t"<<d_norm_<<std::endl;
		std::cout<<"[LineSearch]: Merit of last rollout:\t"<<lowestCost_<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

		alphaBest_ = performLineSearch();

#ifdef DEBUG_PRINT_LINESEARCH
		std::cout<<"[LineSearch]: Best control found at alpha: "<<alphaBest_<<", with trade-off "<<std::endl;
		std::cout<<"[LineSearch]: Cost:\t"<<intermediateCostBest_ + finalCostBest_<<std::endl;
		std::cout<<"[LineSearch]: Defect:\t"<<d_norm_<<std::endl;
#endif //DEBUG_PRINT_LINESEARCH

		if (alphaBest_ == 0.0)
		{
#ifdef DEBUG_PRINT
			std::cout<<"WARNING: No better control found during line search. Converged."<<std::endl;
#endif
			return false;
		}
	}

	if ( fabs((lowestCostPrevious - lowestCost_)/lowestCostPrevious) > settings_.min_cost_improvement)
		return true; //! found better cost

#ifdef DEBUG_PRINT
	std::cout<<"CONVERGED: Cost last iteration: "<<lowestCostPrevious<<", current cost: "<< lowestCost_ << std::endl;
	std::cout<<"CONVERGED: Cost improvement ratio was: "<<fabs(lowestCostPrevious - lowestCost_)/lowestCostPrevious <<"x, which is lower than convergence criteria: "<<settings_.min_cost_improvement<<std::endl;
#endif //DEBUG_PRINT
	return false;
}



template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::executeLineSearchMultipleShooting(
		const size_t threadId,
		const scalar_t alpha,
		const ControlVectorArray& u_ff_update,
		const StateVectorArray& x_update,
		ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_alpha,
		ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_shot_alpha,
		ct::core::StateVectorArray<STATE_DIM, SCALAR>& defects_recorded,
		ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_alpha,
		scalar_t& intermediateCost,
		scalar_t& finalCost,
		scalar_t& defectNorm,
		std::atomic_bool* terminationFlag
) const
{
	intermediateCost =  std::numeric_limits<scalar_t>::infinity();
	finalCost = std::numeric_limits<scalar_t>::infinity();
	defectNorm = std::numeric_limits<scalar_t>::infinity();

	if (terminationFlag && *terminationFlag) return;


	//! update feedforward
	for (int k=K_-1; k>=0; k--)
	{
		u_alpha[k] = alpha * u_ff_update[k] + u_ff_prev_[k];
	}

	//! update state decision variables
	for (int k=K_; k>=0; k--)
	{
		x_alpha[k] = alpha * x_update[k] + x_prev_[k];
	}

	if (terminationFlag && *terminationFlag) return;

	// compute costs
	computeCostsOfTrajectory(threadId, x_alpha, u_alpha, intermediateCost, finalCost);

	if (terminationFlag && *terminationFlag) return;

	// rollout shots
	rolloutShotsSingleThreaded(threadId, 0, K_-1, u_alpha, x_alpha, x_shot_alpha, defects_recorded);

	if (terminationFlag && *terminationFlag) return;

	// compute defects norm
	defectNorm = computeDefectsNorm<1>(defects_recorded);

	// form a merit from that

	if (terminationFlag && *terminationFlag) return;
}




template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::prepareSolveLQProblem()
{
	// if solver is HPIPM, there's nothing to prepare
	if(settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::HPIPM_SOLVER)
	{}
	// if solver is GNRiccati - we iterate backward up to the first stage
	else if(settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
	{
		lqocProblem_->x_ = x_;
		lqocProblem_->u_ = u_ff_;
		lqocSolver_->setProblem(lqocProblem_);

		//iterate backward up to first stage
		for (int i=this->lqocProblem_->getNumberOfStages()-1; i>=1; i--)
			lqocSolver_->solveSingleStage(i);
	}
	else
		throw std::runtime_error("unknown solver type in prepareSolveLQProblem()");
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::finishSolveLQProblem()
{
	// if solver is HPIPM, solve the full problem
	if(settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::HPIPM_SOLVER)
	{
		solveFullLQProblem();
	}
	else if(settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
	{
		// if solver is GNRiccati, solve the first stage and get solution
		lqocProblem_->x_ = x_;
		lqocProblem_->u_ = u_ff_;
		lqocSolver_->setProblem(lqocProblem_);
		lqocSolver_->solveSingleStage(0);
		lqocSolver_->computeStateAndControlUpdates();
	}
	else
		throw std::runtime_error("unknown solver type in finishSolveLQProblem()");
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::solveFullLQProblem()
{
	lqocProblem_->x_ = x_;
	lqocProblem_->u_ = u_ff_;
	lqocSolver_->setProblem(lqocProblem_);
	lqocSolver_->solve();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::updateCosts()
{
	intermediateCostPrevious_ = intermediateCostBest_;
	finalCostPrevious_ = finalCostBest_;
	computeCostsOfTrajectory(settings_.nThreads, x_, u_ff_, intermediateCostBest_, finalCostBest_);
}


//template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
//bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::isConverged()
//{
//	//! check if sum of norm of all defects is smaller than convergence criterion
//	if (d_norm_ > settings_.maxDefectSum)
//		return false;
//
//	SCALAR previousCost = intermediateCostPrevious_ + finalCostPrevious_;
//	SCALAR newCost = intermediateCostBest_ + finalCostBest_;
//
//	if ( fabs((previousCost - newCost)/previousCost) > settings_.min_cost_improvement)
//		return false;
//
//	return true;
//}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
template <typename ARRAY_TYPE, size_t ORDER>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDiscreteArrayNorm(const ARRAY_TYPE& d) const
{
	SCALAR norm = 0.0;

	for (int k=0; k<d.size(); k++)
	{
		norm += d[k].template lpNorm<ORDER>();
	}
	return norm;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
template <typename ARRAY_TYPE, size_t ORDER>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDiscreteArrayNorm(const ARRAY_TYPE& a, const ARRAY_TYPE& b) const
{
	assert(a.size() == b.size());

	SCALAR norm = 0.0;

	for (int k=0; k<a.size(); k++)
	{
		norm += (a[k]-b[k]).template lpNorm<ORDER>();
	}
	return norm;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::retrieveLastLinearizedModel(StateMatrixArray& A, StateControlMatrixArray& B)
{
	A = lqocProblem_->A_;
	B = lqocProblem_->B_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::Policy_t& NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSolution()
{
	policy_.update(x_, u_ff_, L_, t_);
	return policy_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::reset()
{
	firstRollout_ = true;
	iteration_ = 0;
	d_norm_ = std::numeric_limits<scalar_t>::infinity();
	lx_norm_ = std::numeric_limits<scalar_t>::infinity();
	lu_norm_ = std::numeric_limits<scalar_t>::infinity();
	intermediateCostBest_ = std::numeric_limits<scalar_t>::infinity();
	finalCostBest_ = std::numeric_limits<scalar_t>::infinity();
	intermediateCostPrevious_ = std::numeric_limits<scalar_t>::infinity();
	finalCostPrevious_ = std::numeric_limits<scalar_t>::infinity();
}


} //namespace optcon
} //namespace ct
