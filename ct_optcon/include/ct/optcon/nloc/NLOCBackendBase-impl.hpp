/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define SYMPLECTIC_ENABLED        \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V > 0 && P > 0 && (V+P==ST)), void>::type
#define SYMPLECTIC_DISABLED       \
    template <size_t V, size_t P, size_t ST> \
    typename std::enable_if<(V <= 0 || P <= 0 || (V+P!=ST)), void>::type

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::NLOCBackendBase(
    const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
    const Settings_t& settings)
    :

      substepRecorders_(),
      integrators_(settings.nThreads + 1),
      sensitivity_(settings.nThreads + 1),
      integratorsEulerSymplectic_(settings.nThreads + 1),
      integratorsRkSymplectic_(settings.nThreads + 1),

      controller_(settings.nThreads + 1),
      initialized_(false),
      configured_(false),
      iteration_(0),
      settings_(settings),
      K_(0),
      d_norm_(0.0),
      e_box_norm_(0.0),
      e_gen_norm_(0.0),
      lx_norm_(0.0),
      lu_norm_(0.0),
      lqocProblem_(new LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR>()),

      substepsX_(new StateSubsteps),
      substepsU_(new ControlSubsteps),

      intermediateCostBest_(std::numeric_limits<SCALAR>::infinity()),
      finalCostBest_(std::numeric_limits<SCALAR>::infinity()),
      lowestCost_(std::numeric_limits<SCALAR>::infinity()),
      intermediateCostPrevious_(std::numeric_limits<SCALAR>::infinity()),
      finalCostPrevious_(std::numeric_limits<SCALAR>::infinity()),
      linearSystems_(settings.nThreads + 1),
      boxConstraints_(settings.nThreads + 1, nullptr),      // initialize constraints with null
      generalConstraints_(settings.nThreads + 1, nullptr),  // initialize constraints with null
      firstRollout_(true),
      alphaBest_(-1),
      lqpCounter_(0)
{
    Eigen::initParallel();

    for (int i = 0; i < settings.nThreads + 1; i++)
    {
        controller_[i] = ConstantControllerPtr(new core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>());
    }

    configure(settings);

    changeTimeHorizon(optConProblem.getTimeHorizon());
    changeInitialState(optConProblem.getInitialState());
    changeCostFunction(optConProblem.getCostFunction());
    changeNonlinearSystem(optConProblem.getNonlinearSystem());
    changeLinearSystem(optConProblem.getLinearSystem());

    // if the optimal problem has constraints, change
    if (optConProblem.getBoxConstraints())
        changeBoxConstraints(optConProblem.getBoxConstraints());
    if (optConProblem.getGeneralConstraints())
        changeGeneralConstraints(optConProblem.getGeneralConstraints());
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::NLOCBackendBase(
    const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR>& optConProblem,
    const std::string& settingsFile,
    bool verbose,
    const std::string& ns)
    : NLOCBackendBase(optConProblem, Settings_t::fromConfigFile(settingsFile, verbose, ns))
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::~NLOCBackendBase()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setInitialGuess(const Policy_t& initialGuess)
{
    if (initialGuess.x_ref().size() < (size_t)K_ + 1)
    {
        std::cout << "Initial state guess length too short. Received length " << initialGuess.x_ref().size()
                  << ", expected " << K_ + 1 << std::endl;
        throw(std::runtime_error("initial state guess to short"));
    }

    if (initialGuess.uff().size() < (size_t)K_)
    {
        std::cout << "Initial control guess length too short. Received length " << initialGuess.uff().size()
                  << ", expected " << K_ << std::endl;
        throw std::runtime_error("initial control guess to short");
    }

    if (initialGuess.K().size() < (size_t)K_)
    {
        std::cout << "Initial feedback length too short. Received length " << initialGuess.K().size() << ", expected "
                  << K_ << std::endl;
        throw std::runtime_error("initial control guess to short");
    }

    u_ff_ = initialGuess.uff();
    L_ = initialGuess.K();
    x_ = initialGuess.x_ref();
    x_prev_ = x_;

    if (x_.size() > (size_t)K_ + 1)
    {
        std::cout << "Warning, initial state guess too long. Received length " << x_.size() << ", expected " << K_ + 1
                  << ", will truncate" << std::endl;
        x_.resize(K_ + 1);
    }

    if (u_ff_.size() > (size_t)K_)
    {
        std::cout << "Warning, initial control guess too long. Received length " << u_ff_.size() << ", expected " << K_
                  << ", will truncate" << std::endl;
        u_ff_.resize(K_);
    }

    if (L_.size() > (size_t)K_)
    {
        std::cout << "Warning, initial feedback guess too long. Received length " << L_.size() << ", expected " << K_
                  << ", will truncate" << std::endl;
        L_.resize(K_);
    }

    initialized_ = true;

    t_ = TimeArray(settings_.dt, x_.size(), 0.0);

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

    int newK = settings_.computeK(tf);

    if (newK == K_)
        return;

    K_ = newK;

    t_ = TimeArray(settings_.dt, K_ + 1, 0.0);

    lx_.resize(K_ + 1);
    x_.resize(K_ + 1);
    x_prev_.resize(K_ + 1);
    xShot_.resize(K_ + 1);

    lu_.resize(K_);
    u_ff_.resize(K_);
    u_ff_prev_.resize(K_);
    L_.resize(K_);

    substepsX_->resize(K_ + 1);
    substepsU_->resize(K_ + 1);

    lqocProblem_->changeNumStages(K_);
    lqocProblem_->setZero();

    lqocSolver_->setProblem(lqocProblem_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeInitialState(
    const core::StateVector<STATE_DIM, SCALAR>& x0)
{
    if (x_.size() == 0)
        x_.resize(1);

    x_[0] = x0;
    reset();  // since initial state changed, we have to start fresh, i.e. with a rollout
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeCostFunction(
    const typename OptConProblem_t::CostFunctionPtr_t& cf)
{
    if (cf == nullptr)
        throw std::runtime_error("cost function is nullptr");

    costFunctions_.resize(settings_.nThreads + 1);

    for (int i = 0; i < settings_.nThreads + 1; i++)
    {
        // make a deep copy
        costFunctions_[i] = typename OptConProblem_t::CostFunctionPtr_t(cf->clone());
    }

    // recompute cost if line search is active
    // TODO: this should be multi-threaded to save time
    if (iteration_ > 0 && settings_.lineSearchSettings.active)
        computeCostsOfTrajectory(settings_.nThreads, x_, u_ff_, intermediateCostBest_, finalCostBest_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeNonlinearSystem(
    const typename OptConProblem_t::DynamicsPtr_t& dyn)
{
    if (dyn == nullptr)
        throw std::runtime_error("system dynamics are nullptr");

    systems_.resize(settings_.nThreads + 1);
    substepRecorders_.resize(settings_.nThreads + 1);
    integrators_.resize(settings_.nThreads + 1);
    sensitivity_.resize(settings_.nThreads + 1);
    integratorsEulerSymplectic_.resize(settings_.nThreads + 1);
    integratorsRkSymplectic_.resize(settings_.nThreads + 1);

    for (int i = 0; i < settings_.nThreads + 1; i++)
    {
        // make a deep copy
        systems_[i] = typename OptConProblem_t::DynamicsPtr_t(dyn->clone());
        systems_[i]->setController(controller_[i]);

        substepRecorders_[i] = std::shared_ptr<ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new ct::core::SubstepRecorder<STATE_DIM, CONTROL_DIM, SCALAR>(systems_[i]));

        if (controller_[i] == nullptr)
            throw std::runtime_error("Controller not defined");

        // if symplectic integrator then don't create normal ones
        if (settings_.integrator != ct::core::IntegrationType::EULER_SYM &&
            settings_.integrator != ct::core::IntegrationType::RK_SYM)
        {
            integrators_[i] = std::shared_ptr<ct::core::Integrator<STATE_DIM, SCALAR>>(
                new ct::core::Integrator<STATE_DIM, SCALAR>(systems_[i], settings_.integrator, substepRecorders_[i]));
        }
        initializeSymplecticIntegrators<V_DIM, P_DIM, STATE_DIM>(i);


        if (settings_.useSensitivityIntegrator)
        {
            if (settings_.integrator != ct::core::IntegrationType::EULER &&
                settings_.integrator != ct::core::IntegrationType::EULERCT &&
                settings_.integrator != ct::core::IntegrationType::RK4 &&
                settings_.integrator != ct::core::IntegrationType::RK4CT &&
                settings_.integrator != ct::core::IntegrationType::EULER_SYM)
                throw std::runtime_error("sensitivity integrator only available for Euler and RK4 integrators");

            sensitivity_[i] =
                SensitivityPtr(new ct::core::SensitivityIntegrator<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(
                    settings_.getSimulationTimestep(), linearSystems_[i], controller_[i], settings_.integrator,
                    settings_.timeVaryingDiscretization));
        }
        else
        {
            sensitivity_[i] =
                SensitivityPtr(new ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(
                    settings_.dt, linearSystems_[i], settings_.discretization));
        }
    }
    reset();  // since system changed, we have to start fresh, i.e. with a rollout
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeBoxConstraints(
    const typename OptConProblem_t::ConstraintPtr_t& con)
{
    if (con == nullptr)
        throw std::runtime_error("NLOCBackendBase: box constraint is nullptr");

    boxConstraints_.resize(settings_.nThreads + 1);

    for (int i = 0; i < settings_.nThreads + 1; i++)
    {
        // make a deep copy
        boxConstraints_[i] = typename OptConProblem_t::ConstraintPtr_t(con->clone());
    }

    // TODO can we do this multi-threaded?
    if (iteration_ > 0 && settings_.lineSearchSettings.active)
        computeBoxConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_box_norm_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeGeneralConstraints(
    const typename OptConProblem_t::ConstraintPtr_t& con)
{
    if (con == nullptr)
        throw std::runtime_error("NLOCBackendBase: general constraint is nullptr");

    generalConstraints_.resize(settings_.nThreads + 1);

    for (int i = 0; i < settings_.nThreads + 1; i++)
    {
        // make a deep copy
        generalConstraints_[i] = typename OptConProblem_t::ConstraintPtr_t(con->clone());
    }

    // TODO can we do this multi-threaded?
    if (iteration_ > 0 && settings_.lineSearchSettings.active)
        computeGeneralConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_gen_norm_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_ENABLED NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeSymplecticIntegrators(
    size_t i)
{
    if (systems_[i]->isSymplectic())
    {
        //! it only makes sense to compile the following code, if V_DIM > 0 and P_DIM > 0
        integratorsEulerSymplectic_[i] =
            std::shared_ptr<ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                new ct::core::IntegratorSymplecticEuler<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                    std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                        systems_[i]),
                    substepRecorders_[i]));
        integratorsRkSymplectic_[i] =
            std::shared_ptr<ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                new ct::core::IntegratorSymplecticRk<P_DIM, V_DIM, CONTROL_DIM, SCALAR>(
                    std::static_pointer_cast<ct::core::SymplecticSystem<P_DIM, V_DIM, CONTROL_DIM, SCALAR>>(
                        systems_[i])));
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_DISABLED NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeSymplecticIntegrators(
    size_t i)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeLinearSystem(
    const typename OptConProblem_t::LinearPtr_t& lin)
{
    linearSystems_.resize(settings_.nThreads + 1);

    for (int i = 0; i < settings_.nThreads + 1; i++)
    {
        // make a deep copy
        linearSystems_[i] = typename OptConProblem_t::LinearPtr_t(lin->clone());

        sensitivity_[i]->setLinearSystem(linearSystems_[i]);
    }
    // technically a linear system change does not require a new rollout. Hence, we do not reset.
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::checkProblem()
{
    if (K_ == 0)
        throw std::runtime_error("Time horizon too small resulting in 0 steps");

    if (u_ff_.size() < (size_t)K_)
    {
        std::cout << "Provided initial feed forward controller too short, should be at least " << K_ << " but is "
                  << u_ff_.size() << " long." << std::endl;
        throw(std::runtime_error("Provided initial feed forward controller too short"));
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::configure(const Settings_t& settings)
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
    for (int i = 0; i < settings.nThreads + 1; i++)
    {
        if (!sensitivity_[i])
            break;

        sensitivity_[i]->setApproximation(settings.discretization);

        if (settings.useSensitivityIntegrator)
            sensitivity_[i]->setTimeDiscretization(settings.getSimulationTimestep());
        else
            sensitivity_[i]->setTimeDiscretization(settings.dt);
    }

    if (settings.integrator != settings_.integrator)
    {
        throw std::runtime_error("Cannot change integration type.");
    }

    // select the linear quadratic solver based on settings file
    if (settings.lqocp_solver == NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER)
    {
        lqocSolver_ = std::shared_ptr<GNRiccatiSolver<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new GNRiccatiSolver<STATE_DIM, CONTROL_DIM, SCALAR>());
    }
    else if (settings.lqocp_solver == NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER)
    {
#ifdef HPIPM
        lqocSolver_ =
            std::shared_ptr<HPIPMInterface<STATE_DIM, CONTROL_DIM>>(new HPIPMInterface<STATE_DIM, CONTROL_DIM>());
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


/*
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::simpleRollout(
		const size_t threadId,
		const ControlVectorArray& uff,
		const StateVectorArray& x_ref_lqr,
		StateVectorArray& x_local,
		ControlVectorArray& u_recorded
		)const
{
	const double& dt = settings_.dt;
	const double dt_sim = settings_.getSimulationTimestep();
	const size_t subSteps = settings_.K_sim;
	const int K_local = K_;

	u_recorded.clear();

	x_local.resize(K_+1);
	x_local.front() = x_ref_lqr.front();

	if(uff.size() < (size_t)K_) throw std::runtime_error("simpleRollout: u_local is too short.");


	for (int i = 1; i<K_local+1; i++)
	{
		x_local[i] = x_local[i-1];

		u_recorded.push_back(uff[i-1] + L_[i-1] * (x_local[i-1] - x_ref_lqr[i-1]));

		controller_[threadId]->setControl(u_recorded.back());


		if(settings_.integrator == ct::core::IntegrationType::EULER_SYM || settings_.integrator == ct::core::IntegrationType::RK_SYM)
		{
			integrateSymplectic<V_DIM, P_DIM, STATE_DIM>(threadId, x_local[i], 0, subSteps, dt_sim);
		} else
		{
			integrators_[threadId]->integrate_n_steps(x_local[i], 0, subSteps, dt_sim);
		}


		// check if nan
		for (size_t k=0; k<STATE_DIM; k++)
		{
			if (isnan(x_local[i](k)))
			{
				x_local.resize(K_local+1, ct::core::StateVector<STATE_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				u_recorded.resize(K_local, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				return false;
			}
		}
		for (size_t k=0; k<CONTROL_DIM; k++)
		{
			if (isnan(u_recorded.back()(k)))
			{
				x_local.resize(K_local+1, ct::core::StateVector<STATE_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				u_recorded.resize(K_local, ct::core::ControlVector<CONTROL_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
				std::cout << "control unstable" << std::endl;
				return false;
			}
		}
	}
	return true;
}
*/


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::rolloutSingleShot(const size_t threadId,
    const size_t k,  //! the starting index of the shot
    ControlVectorArray& u_local,
    StateVectorArray& x_local,
    const StateVectorArray& x_ref_lqr,
    StateVectorArray& xShot,  //! the value at the end of each integration interval
    StateSubsteps& substepsX,
    ControlSubsteps& substepsU,
    std::atomic_bool* terminationFlag) const
{
    const double& dt = settings_.dt;
    const double dt_sim = settings_.getSimulationTimestep();
    const size_t subSteps = settings_.K_sim;
    const int K_local = K_;

    if (u_local.size() < (size_t)K_)
        throw std::runtime_error("rolloutSingleShot: u_local is too short.");
    if (x_local.size() < (size_t)K_ + 1)
        throw std::runtime_error("rolloutSingleShot: x_local is too short.");
    if (xShot.size() < (size_t)K_ + 1)
        throw std::runtime_error("rolloutSingleShot: xShot is too short.");

    xShot[k] = x_local[k];  // initialize

    //! determine index where to stop at the latest
    int K_stop = k + settings_.K_shot;
    if (K_stop > K_local)
        K_stop = K_local;

    if (settings_.nlocp_algorithm == NLOptConSettings::NLOCP_ALGORITHM::ILQR)
        K_stop = K_local;  //! @todo this is not elegant - need to improve.

    // for each control step
    for (int i = (int)k; i < K_stop; i++)
    {
        if (terminationFlag && *terminationFlag)
            return false;

        substepRecorders_[threadId]->reset();

        if (i > (int)k)
        {
            xShot[i] = xShot[i - 1];  //! initialize integration variable
        }

        // Todo: the order here is not optimal. In some cases, we will overwrite x_ref_lqr immediately in the next step!
        if (settings_.closedLoopShooting)  // overwrite control
            u_local[i] += L_[i] * (xShot[i] - x_ref_lqr[i]);

        //! @todo: here we override the state trajectory directly (as passed by reference). This is bad.
        if (i > (int)k)
        {
            x_local[i] = xShot[i - 1];  //!"overwrite" x_local
        }


        controller_[threadId]->setControl(u_local[i]);


        if (settings_.integrator == ct::core::IntegrationType::EULER_SYM ||
            settings_.integrator == ct::core::IntegrationType::RK_SYM)
        {
            integrateSymplectic<V_DIM, P_DIM, STATE_DIM>(threadId, xShot[i], i * dt, subSteps, dt_sim);
        }
        else
        {
            integrators_[threadId]->integrate_n_steps(xShot[i], i * dt, subSteps, dt_sim);
        }


        if (i == K_local - 1)
            x_local[K_local] = xShot[K_local - 1];  //! fill in terminal state if required


        // check if nan
        for (size_t j = 0; j < STATE_DIM; j++)
        {
            if (std::isnan(x_local[i](j)))
            {
                x_local.resize(K_local + 1,
                    ct::core::StateVector<STATE_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
                u_local.resize(K_local,
                    ct::core::ControlVector<CONTROL_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
                return false;
            }
        }
        for (size_t j = 0; j < CONTROL_DIM; j++)
        {
            if (std::isnan(u_local[i](j)))
            {
                x_local.resize(K_local + 1,
                    ct::core::StateVector<STATE_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
                u_local.resize(K_local,
                    ct::core::ControlVector<CONTROL_DIM, SCALAR>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
                std::cout << "control unstable" << std::endl;
                return false;
            }
        }

        // get substeps for sensitivities
        substepsX[i] = substepRecorders_[threadId]->getSubstates();
        substepsU[i] = substepRecorders_[threadId]->getSubcontrols();
    }

    return true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_ENABLED NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::integrateSymplectic(size_t threadId,
    ct::core::StateVector<STATE_DIM, SCALAR>& x0,
    const double& t,
    const size_t& steps,
    const double& dt_sim) const
{
    if (!systems_[threadId]->isSymplectic())
        throw std::runtime_error("Trying to integrate using symplectic integrator, but system is not symplectic.");

    if (settings_.integrator == ct::core::IntegrationType::EULER_SYM)
    {
        integratorsEulerSymplectic_[threadId]->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else if (settings_.integrator == ct::core::IntegrationType::RK_SYM)
    {
        integratorsRkSymplectic_[threadId]->integrate_n_steps(x0, t, steps, dt_sim);
    }
    else
    {
        throw std::runtime_error("invalid symplectic integrator specified");
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SYMPLECTIC_DISABLED NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::integrateSymplectic(size_t threadId,
    ct::core::StateVector<STATE_DIM, SCALAR>& x0,
    const double& t,
    const size_t& steps,
    const double& dt_sim) const
{
    throw std::runtime_error("Symplectic integrator selected but invalid dimensions for it. Check V_DIM>1, P_DIM>1");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::rolloutShotsSingleThreaded(size_t threadId,
    size_t firstIndex,
    size_t lastIndex,
    ControlVectorArray& u_ff_local,
    StateVectorArray& x_local,
    const StateVectorArray& x_ref_lqr,
    StateVectorArray& xShot,
    StateVectorArray& d,
    StateSubsteps& substepsX,
    ControlSubsteps& substepsU) const
{
    //! make sure all intermediate entries in the defect trajectory are zero
    d.setConstant(state_vector_t::Zero());

    for (size_t k = firstIndex; k <= lastIndex; k = k + settings_.K_shot)
    {
        // first rollout the shot
        bool dynamicsGood = rolloutSingleShot(threadId, k, u_ff_local, x_local, x_ref_lqr, xShot, substepsX, substepsU);

        if (!dynamicsGood)
            return false;

        // then compute the corresponding defect
        computeSingleDefect(k, x_local, xShot, d);
    }
    return true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeSingleDefect(size_t k,
    const StateVectorArray& x_local,
    const StateVectorArray& xShot,
    StateVectorArray& d) const
{
    //! compute the index where the next shot starts (respect total number of stages)
    int k_next = std::min(K_, (int)k + settings_.K_shot);

    if (k_next < K_)
    {
        d[k_next - 1] = xShot[k_next - 1] - x_local[k_next];
    }
    //! else ... all other entries of d remain zero.
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeCostsOfTrajectory(size_t threadId,
    const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
    const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
    scalar_t& intermediateCost,
    scalar_t& finalCost) const
{
    intermediateCost = 0;

    for (size_t k = 0; k < (size_t)K_; k++)
    {
        // feed current state and control to cost function
        costFunctions_[threadId]->setCurrentStateAndControl(x_local[k], u_local[k], settings_.dt * k);

        // derivative of cost with respect to state
        intermediateCost += costFunctions_[threadId]->evaluateIntermediate();
    }
    intermediateCost *= settings_.dt;

    costFunctions_[threadId]->setCurrentStateAndControl(x_local[K_], control_vector_t::Zero(), settings_.dt * K_);
    finalCost = costFunctions_[threadId]->evaluateTerminal();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeBoxConstraintErrorOfTrajectory(
    size_t threadId,
    const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
    const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
    scalar_t& e_tot) const
{
    e_tot = 0;

    // intermediate constraints
    for (size_t k = 0; k < (size_t)K_; k++)
    {
        if (boxConstraints_[threadId] != nullptr)
        {
            if (boxConstraints_[threadId]->getIntermediateConstraintsCount() > 0)
            {
                boxConstraints_[threadId]->setCurrentStateAndControl(x_local[k], u_local[k], settings_.dt * k);
                Eigen::Matrix<SCALAR, -1, 1> box_err = boxConstraints_[threadId]->getTotalBoundsViolationIntermediate();
                e_tot += box_err.norm();  // TODO check if we should use different norms here
            }
        }
    }

    // normalize integrated violation by time
    e_tot *= settings_.dt;

    // terminal constraint violation
    if (boxConstraints_[threadId] != nullptr)
    {
        if (boxConstraints_[threadId]->getTerminalConstraintsCount() > 0)
        {
            boxConstraints_[threadId]->setCurrentStateAndControl(
                x_local[K_], control_vector_t::Zero(), settings_.dt * K_);
            Eigen::Matrix<SCALAR, -1, 1> box_err = boxConstraints_[threadId]->getTotalBoundsViolationTerminal();
            e_tot += box_err.norm();  // TODO check if we should use different norms here
        }
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeGeneralConstraintErrorOfTrajectory(
    size_t threadId,
    const ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
    const ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
    scalar_t& e_tot) const
{
    e_tot = 0;

    // intermediate constraints
    for (size_t k = 0; k < (size_t)K_; k++)
    {
        if (generalConstraints_[threadId] != nullptr)
        {
            if (generalConstraints_[threadId]->getIntermediateConstraintsCount() > 0)
            {
                generalConstraints_[threadId]->setCurrentStateAndControl(x_local[k], u_local[k], settings_.dt * k);
                Eigen::Matrix<SCALAR, -1, 1> gen_err =
                    generalConstraints_[threadId]->getTotalBoundsViolationIntermediate();
                e_tot += gen_err.norm();  // TODO check if we should use different norms here
            }
        }
    }

    // normalize integrated violation by time
    e_tot *= settings_.dt;

    if (generalConstraints_[threadId] != nullptr)
    {
        if (generalConstraints_[threadId]->getTerminalConstraintsCount() > 0)
        {
            generalConstraints_[threadId]->setCurrentStateAndControl(
                x_local[K_], control_vector_t::Zero(), settings_.dt * K_);
            Eigen::Matrix<SCALAR, -1, 1> gen_err = generalConstraints_[threadId]->getTotalBoundsViolationTerminal();
            e_tot += gen_err.norm();  // TODO check if we should use different norms here
        }
    }
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
    const core::ControlVector<CONTROL_DIM, SCALAR> u_last = u_ff_[std::min((int)k + 1, K_ - 1)];

    assert(lqocProblem_ != nullptr);

    assert(&lqocProblem_->A_[k] != nullptr);
    assert(&lqocProblem_->B_[k] != nullptr);

    assert(lqocProblem_->A_.size() > k);
    assert(lqocProblem_->B_.size() > k);

    sensitivity_[threadId]->setSubstepTrajectoryReference(substepsX_.get(), substepsU_.get());
    sensitivity_[threadId]->getAandB(x_[k], u_ff_[k], xShot_[k], (int)k, settings_.K_sim, p.A_[k], p.B_[k]);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setBoxConstraintsForLQOCProblem()
{
    // set box constraints if there are any
    if (boxConstraints_[settings_.nThreads] != nullptr)
    {
        // temp vars
        Eigen::VectorXi foo, u_sparsity_intermediate, x_sparsity_intermediate, x_sparsity_terminal;

        // intermediate box constraints
        const int nb_ux_intermediate = boxConstraints_[settings_.nThreads]->getJacobianStateNonZeroCountIntermediate() +
                                       boxConstraints_[settings_.nThreads]->getJacobianInputNonZeroCountIntermediate();

        if (nb_ux_intermediate > 0)
        {
            boxConstraints_[settings_.nThreads]->sparsityPatternInputIntermediate(foo, u_sparsity_intermediate);
            boxConstraints_[settings_.nThreads]->sparsityPatternStateIntermediate(foo, x_sparsity_intermediate);

            Eigen::VectorXi ux_sparsity_intermediate(nb_ux_intermediate);
            x_sparsity_intermediate.array() += CONTROL_DIM;  // shift indices to match combined decision vector [u, x]
            ux_sparsity_intermediate << u_sparsity_intermediate, x_sparsity_intermediate;

            lqocProblem_->setIntermediateBoxConstraints(nb_ux_intermediate,
                boxConstraints_[settings_.nThreads]->getLowerBoundsIntermediate(),
                boxConstraints_[settings_.nThreads]->getUpperBoundsIntermediate(), ux_sparsity_intermediate);
        }

        // terminal box constraints
        const int nb_x_terminal = boxConstraints_[settings_.nThreads]->getJacobianStateNonZeroCountTerminal();

        if (nb_x_terminal > 0)
        {
            boxConstraints_[settings_.nThreads]->sparsityPatternStateTerminal(foo, x_sparsity_terminal);

            lqocProblem_->setTerminalBoxConstraints(nb_x_terminal,
                boxConstraints_[settings_.nThreads]->getLowerBoundsTerminal(),
                boxConstraints_[settings_.nThreads]->getUpperBoundsTerminal(), x_sparsity_terminal);
        }
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeLinearizedConstraints(size_t threadId,
    size_t k)
{
    // set general if there are any
    if (generalConstraints_[threadId] != nullptr)
    {
        LQOCProblem_t& p = *lqocProblem_;
        const scalar_t& dt = settings_.dt;

        // treat general constraints
        generalConstraints_[threadId]->setCurrentStateAndControl(x_[k], u_ff_[k], dt * k);

        p.ng_[k] = generalConstraints_[threadId]->getIntermediateConstraintsCount();
        if (p.ng_[k] > 0)
        {
            p.hasGenConstraints_ = true;
            p.C_[k] = generalConstraints_[threadId]->jacobianStateIntermediate();
            p.D_[k] = generalConstraints_[threadId]->jacobianInputIntermediate();

            Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> g_eval = generalConstraints_[threadId]->evaluateIntermediate();

            // rewrite constraint in absolute coordinates as required by LQOC problem
            p.d_lb_[k] = generalConstraints_[threadId]->getLowerBoundsIntermediate() - g_eval + p.C_[k] * x_[k] +
                         p.D_[k] * u_ff_[k];
            p.d_ub_[k] = generalConstraints_[threadId]->getUpperBoundsIntermediate() - g_eval + p.C_[k] * x_[k] +
                         p.D_[k] * u_ff_[k];
        }
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeQuadraticCosts(size_t threadId, size_t k)
{
    LQOCProblem_t& p = *lqocProblem_;
    const scalar_t& dt = settings_.dt;

    // feed current state and control to cost function
    costFunctions_[threadId]->setCurrentStateAndControl(x_[k], u_ff_[k], dt * k);

    // derivative of cost with respect to state
    p.q_[k] = costFunctions_[threadId]->evaluateIntermediate() * dt;

    p.qv_[k] = costFunctions_[threadId]->stateDerivativeIntermediate() * dt;

    p.Q_[k] = costFunctions_[threadId]->stateSecondDerivativeIntermediate() * dt;

    // derivative of cost with respect to control and state
    p.P_[k] = costFunctions_[threadId]->stateControlDerivativeIntermediate() * dt;

    // derivative of cost with respect to control
    p.rv_[k] = costFunctions_[threadId]->controlDerivativeIntermediate() * dt;

    p.R_[k] = costFunctions_[threadId]->controlSecondDerivativeIntermediate() * dt;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initializeCostToGo()
{
    LQOCProblem_t& p = *lqocProblem_;

    // feed current state and control to cost function
    costFunctions_[settings_.nThreads]->setCurrentStateAndControl(x_[K_], control_vector_t::Zero(), settings_.dt * K_);

    // derivative of terminal cost with respect to state
    p.q_[K_] = costFunctions_[settings_.nThreads]->evaluateTerminal();
    p.qv_[K_] = costFunctions_[settings_.nThreads]->stateDerivativeTerminal();
    p.Q_[K_] = costFunctions_[settings_.nThreads]->stateSecondDerivativeTerminal();

    // init terminal general constraints, if any
    if (generalConstraints_[settings_.nThreads] != nullptr)
    {
        p.ng_[K_] = generalConstraints_[settings_.nThreads]->getTerminalConstraintsCount();
        if (p.ng_[K_] > 0)
        {
            p.hasGenConstraints_ = true;
            p.C_[K_] = generalConstraints_[settings_.nThreads]->jacobianStateTerminal();
            p.D_[K_] = generalConstraints_[settings_.nThreads]->jacobianInputTerminal();

            Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> g_eval =
                generalConstraints_[settings_.nThreads]->evaluateTerminal();

            p.d_lb_[K_] =
                generalConstraints_[settings_.nThreads]->getLowerBoundsTerminal() - g_eval + p.C_[K_] * x_[K_];
            p.d_ub_[K_] =
                generalConstraints_[settings_.nThreads]->getUpperBoundsTerminal() - g_eval + p.C_[K_] * x_[K_];
        }
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::printSummary()
{
    SCALAR d_norm_l1 = computeDefectsNorm<1>(lqocProblem_->b_);
    SCALAR d_norm_l2 = computeDefectsNorm<2>(lqocProblem_->b_);
    SCALAR totalCost = intermediateCostBest_ + finalCostBest_;
    SCALAR totalMerit = intermediateCostBest_ + finalCostBest_ + settings_.meritFunctionRho * d_norm_l1;

    SCALAR smallestEigenvalue = 0.0;
    if (settings_.recordSmallestEigenvalue && settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
    {
        smallestEigenvalue = lqocSolver_->getSmallestEigenvalue();
    }

    computeBoxConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_box_norm_);
    computeGeneralConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_gen_norm_);

    summaryAllIterations_.iterations.push_back(iteration_);
    summaryAllIterations_.defect_l1_norms.push_back(d_norm_l1);
    summaryAllIterations_.defect_l2_norms.push_back(d_norm_l2);
    summaryAllIterations_.e_box_norms.push_back(e_box_norm_);
    summaryAllIterations_.e_gen_norms.push_back(e_gen_norm_);
    summaryAllIterations_.lx_norms.push_back(lx_norm_);
    summaryAllIterations_.lu_norms.push_back(lu_norm_);
    summaryAllIterations_.intermediateCosts.push_back(intermediateCostBest_);
    summaryAllIterations_.finalCosts.push_back(finalCostBest_);
    summaryAllIterations_.totalCosts.push_back(totalCost);
    summaryAllIterations_.merits.push_back(totalMerit);
    summaryAllIterations_.stepSizes.push_back(alphaBest_);
    summaryAllIterations_.smallestEigenvalues.push_back(smallestEigenvalue);

    if (settings_.printSummary)
        summaryAllIterations_.printSummaryLastIteration();

    //! @todo the printing of the smallest eigenvalue is hacky
    if (settings_.printSummary && settings_.recordSmallestEigenvalue &&
        settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
    {
        std::cout << std::setprecision(15) << "smallest eigenvalue this iteration: " << smallestEigenvalue << std::endl;
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logToMatlab(const size_t& iteration)
{
#ifdef MATLAB_FULL_LOG

    std::cout << "Logging to Matlab" << std::endl;

    LQOCProblem_t& p = *lqocProblem_;

    matFile_.open(settings_.loggingPrefix + "Log" + std::to_string(iteration) + ".mat");

    matFile_.put("iteration", iteration);
    matFile_.put("K", K_);
    matFile_.put("dt", settings_.dt);
    matFile_.put("K_sim", settings_.K_sim);
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

    computeBoxConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_box_norm_);
    computeGeneralConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_gen_norm_);
    matFile_.put("e_box_norm", e_box_norm_);
    matFile_.put("e_gen_norm", e_gen_norm_);

    matFile_.close();
#endif  //MATLAB_FULL_LOG
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logInitToMatlab()
{
#ifdef MATLAB

    std::cout << "Logging init guess to Matlab" << std::endl;

    matFile_.open(settings_.loggingPrefix + "LogInit.mat");

    matFile_.put("K", K_);
    matFile_.put("dt", settings_.dt);
    matFile_.put("K_sim", settings_.K_sim);

    matFile_.put("x", x_.toImplementation());
    matFile_.put("u_ff", u_ff_.toImplementation());
    matFile_.put("d", lqocProblem_->b_.toImplementation());
    matFile_.put("cost", getCost());

    d_norm_ = computeDefectsNorm<1>(lqocProblem_->b_);
    matFile_.put("d_norm", d_norm_);

    computeBoxConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_box_norm_);
    computeGeneralConstraintErrorOfTrajectory(settings_.nThreads, x_, u_ff_, e_gen_norm_);
    matFile_.put("e_box_norm", e_box_norm_);
    matFile_.put("e_gen_norm", e_gen_norm_);

    matFile_.close();
#endif
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::ControlTrajectory<CONTROL_DIM, SCALAR>
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getControlTrajectory() const
{
    // \todo this method currently copies the time array (suboptimal)

    core::tpl::TimeArray<SCALAR> t_control = t_;
    t_control.pop_back();

    return core::ControlTrajectory<CONTROL_DIM, SCALAR>(t_control, u_ff_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::StateTrajectory<STATE_DIM, SCALAR>
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getStateTrajectory() const
{
    //! \todo this method currently copies the time array (suboptimal)

    core::tpl::TimeArray<SCALAR> t_control = t_;

    return core::StateTrajectory<STATE_DIM, SCALAR>(t_control, x_);
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
    u_ff_prev_ = u_ff_;  // todo note this might be redundant
    x_prev_ = x_;        // todo note this might be redundant

    if (!settings_.lineSearchSettings.active)
    {
        StateVectorArray x_ref_lqr_local(K_ + 1);
        ControlVectorArray uff_local(K_);

        uff_local = u_ff_ + lu_;          //  add lu
        x_ref_lqr_local = x_prev_ + lx_;  // stabilize around current solution candidate

        bool dynamicsGood = rolloutSingleShot(
            settings_.nThreads, 0, uff_local, x_, x_ref_lqr_local, xShot_, *this->substepsX_, *this->substepsU_);

        if (dynamicsGood)
        {
            intermediateCostBest_ = std::numeric_limits<scalar_t>::max();
            finalCostBest_ = std::numeric_limits<scalar_t>::max();
            computeCostsOfTrajectory(settings_.nThreads, x_, uff_local, intermediateCostBest_, finalCostBest_);
            lowestCost_ = intermediateCostBest_ + finalCostBest_;

            if (settings_.printSummary)
            {
                //! compute l2 norms of state and control update
                lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(u_ff_prev_, uff_local);
                lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(x_prev_, x_);
            }
            else
            {
#ifdef MATLAB  // in case of no debug printing but still logging, need to compute them \
    //! compute l2 norms of state and control update
                lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(u_ff_prev_, uff_local);
                lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(x_prev_, x_);
#endif
            }

            x_prev_ = x_;
            u_ff_.swap(uff_local);
            alphaBest_ = 1;
        }
        else
        {
            if (settings_.debugPrint)
            {
                std::cout << "CONVERGED: System became unstable!" << std::endl;
            }
            return false;
        }
    }
    else
    {
        // merit of previous trajectory
        lowestCost_ = intermediateCostBest_ + finalCostBest_ +
                      (e_box_norm_ + e_gen_norm_) * settings_.meritFunctionRhoConstraints;
        lowestCostPrevious = lowestCost_;

        if (settings_.lineSearchSettings.debugPrint)
        {
            std::cout << "[LineSearch]: Starting line search." << std::endl;
            std::cout << "[LineSearch]: Merit last rollout: " << lowestCost_ << std::endl;
        }

        alphaBest_ = performLineSearch();

        if (settings_.lineSearchSettings.debugPrint)
        {
            std::cout << "[LineSearch]: Best control found at alpha: " << alphaBest_ << " . Will use this control."
                      << std::endl;
        }

        if (alphaBest_ == 0.0)
        {
            if (settings_.debugPrint)
            {
                {
                    std::cout << "WARNING: No better control found. Converged." << std::endl;
                }
                return false;
            }
        }
    }

    if ((fabs((lowestCostPrevious - lowestCost_) / lowestCostPrevious)) > settings_.min_cost_improvement)
    {
        return true;  //! found better cost
    }

    if (settings_.debugPrint)
    {
        std::cout << "CONVERGED: Cost last iteration: " << lowestCostPrevious << ", current cost: " << lowestCost_
                  << std::endl;
        std::cout << "CONVERGED: Cost improvement ratio was: "
                  << (lowestCostPrevious - lowestCost_) / lowestCostPrevious
                  << "x, which is lower than convergence criteria: " << settings_.min_cost_improvement << std::endl;
    }
    return false;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::executeLineSearchSingleShooting(
    const size_t threadId,
    const scalar_t alpha,
    ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_local,
    ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>& u_local,
    scalar_t& intermediateCost,
    scalar_t& finalCost,
    scalar_t& e_box_norm,
    scalar_t& e_gen_norm,
    StateSubsteps& substepsX,
    ControlSubsteps& substepsU,
    std::atomic_bool* terminationFlag) const
{
    intermediateCost = std::numeric_limits<scalar_t>::infinity();
    finalCost = std::numeric_limits<scalar_t>::infinity();
    e_box_norm = 0.0;
    e_gen_norm = 0.0;

    if (terminationFlag && *terminationFlag)
        return;

    StateVectorArray x_ref_lqr;
    StateVectorArray xShot_local(K_ + 1);  //! note this is currently only a dummy (\todo nicer solution?)

    //! if stabilizing about new solution candidate, choose lu as feedforward increment and also increment x_prev_ by lx
    u_local = lu_ * alpha + u_ff_prev_;
    x_ref_lqr = lx_ * alpha + x_prev_;

    bool dynamicsGood =
        rolloutSingleShot(threadId, 0, u_local, x_local, x_ref_lqr, xShot_local, substepsX, substepsU, terminationFlag);

    if (terminationFlag && *terminationFlag)
        return;

    if (dynamicsGood)
    {
        // compute cost specific to this alpha
        computeCostsOfTrajectory(threadId, x_local, u_local, intermediateCost, finalCost);

        // compute constraint violations specific to this alpha
        if (boxConstraints_[threadId] != nullptr)
            computeBoxConstraintErrorOfTrajectory(threadId, x_local, u_local, e_box_norm);
        if (generalConstraints_[threadId] != nullptr)
            computeGeneralConstraintErrorOfTrajectory(threadId, x_local, u_local, e_gen_norm);
    }
    else
    {
        if (settings_.debugPrint)
        {
            std::string msg = std::string("dynamics not good, thread: ") + std::to_string(threadId);
            std::cout << msg << std::endl;
        }
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


    if (!settings_.lineSearchSettings.active)  //! do full step updates
    {
        //! lowest cost is cost of last rollout
        lowestCostPrevious = intermediateCostBest_ + finalCostBest_;

        //! update control and states
        doFullStepUpdate();

        resetDefects();

        rolloutShots(0, K_ - 1);

        updateCosts();

        lowestCost_ = intermediateCostBest_ + finalCostBest_;

        if (settings_.printSummary)
        {
            lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(u_ff_prev_, u_ff_);
            lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(x_prev_, x_);
        }
        else
        {
#ifdef MATLAB
            lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(u_ff_prev_, u_ff_);
            lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(x_prev_, x_);
#endif
        }
        x_prev_ = x_;
        alphaBest_ = 1;
    }
    else  //! do line search over a merit function trading off costs and constraint violations
    {
        // merit of previous trajectory
        lowestCost_ = intermediateCostBest_ + finalCostBest_ + d_norm_ * settings_.meritFunctionRho;
        lowestCostPrevious = lowestCost_;

        if (settings_.lineSearchSettings.debugPrint)
        {
            std::cout << "[LineSearch]: Starting line search." << std::endl;
            std::cout << "[LineSearch]: Cost of last rollout:\t" << intermediateCostBest_ + finalCostBest_ << std::endl;
            std::cout << "[LineSearch]: Defect norm last rollout:\t" << d_norm_ << std::endl;
            std::cout << "[LineSearch]: Merit of last rollout:\t" << lowestCost_ << std::endl;
        }

        alphaBest_ = performLineSearch();

        if (settings_.lineSearchSettings.debugPrint)
        {
            std::cout << "[LineSearch]: Best control found at alpha: " << alphaBest_ << ", with trade-off "
                      << std::endl;
            std::cout << "[LineSearch]: Cost:\t" << intermediateCostBest_ + finalCostBest_ << std::endl;
            std::cout << "[LineSearch]: Defect:\t" << d_norm_ << std::endl;
        }

        if (alphaBest_ == 0.0)
        {
            if (settings_.debugPrint || settings_.printSummary)
            {
                std::cout << "WARNING: No better control found during line search. Converged." << std::endl;
            }
            return false;
        }
    }

    if ((fabs((lowestCostPrevious - lowestCost_) / lowestCostPrevious)) > settings_.min_cost_improvement)
    {
        return true;  //! found better cost
    }

    if (settings_.debugPrint)
    {
        std::cout << "CONVERGED: Cost last iteration: " << lowestCostPrevious << ", current cost: " << lowestCost_
                  << std::endl;
        std::cout << "CONVERGED: Cost improvement ratio was: "
                  << fabs(lowestCostPrevious - lowestCost_) / lowestCostPrevious
                  << "x, which is lower than convergence criteria: " << settings_.min_cost_improvement << std::endl;
    }
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
    scalar_t& e_box_norm,
    scalar_t& e_gen_norm,
    StateSubsteps& substepsX,
    ControlSubsteps& substepsU,
    std::atomic_bool* terminationFlag) const
{
    intermediateCost = std::numeric_limits<scalar_t>::max();
    finalCost = std::numeric_limits<scalar_t>::max();
    defectNorm = std::numeric_limits<scalar_t>::max();
    e_box_norm = 0.0;
    e_gen_norm = 0.0;

    if (terminationFlag && *terminationFlag)
        return;

    //! update feedforward
    u_alpha = u_ff_update * alpha + u_ff_prev_;

    //! update state decision variables
    x_alpha = x_update * alpha + x_prev_;


    if (terminationFlag && *terminationFlag)
        return;

    bool dynamicsGood = rolloutShotsSingleThreaded(
        threadId, 0, K_ - 1, u_alpha, x_alpha, x_alpha, x_shot_alpha, defects_recorded, substepsX, substepsU);

    if (terminationFlag && *terminationFlag)
        return;

    //! compute defects norm
    defectNorm = computeDefectsNorm<1>(defects_recorded);

    if (terminationFlag && *terminationFlag)
        return;

    //! compute costs
    if (dynamicsGood)
    {
        computeCostsOfTrajectory(threadId, x_alpha, u_alpha, intermediateCost, finalCost);

        // compute constraint violations specific to this alpha
        if (boxConstraints_[threadId] != nullptr)
            computeBoxConstraintErrorOfTrajectory(threadId, x_alpha, u_alpha, e_box_norm);
        if (generalConstraints_[threadId] != nullptr)
            computeGeneralConstraintErrorOfTrajectory(threadId, x_alpha, u_alpha, e_gen_norm);
    }
    else
    {
        if (settings_.debugPrint)
        {
            std::string msg = std::string("dynamics not good, thread: ") + std::to_string(threadId);
            std::cout << msg << std::endl;
        }
    }

    if (terminationFlag && *terminationFlag)
        return;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::prepareSolveLQProblem(size_t startIndex)
{
    lqpCounter_++;

    // if solver is HPIPM, there's nothing to prepare
    if (settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::HPIPM_SOLVER)
    {
    }
    // if solver is GNRiccati - we iterate backward up to the first stage
    else if (settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
    {
        lqocProblem_->x_ = x_;
        lqocProblem_->u_ = u_ff_;
        lqocSolver_->setProblem(lqocProblem_);

        //iterate backward up to first stage
        for (int i = this->lqocProblem_->getNumberOfStages() - 1; i >= startIndex; i--)
            lqocSolver_->solveSingleStage(i);
    }
    else
        throw std::runtime_error("unknown solver type in prepareSolveLQProblem()");
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::finishSolveLQProblem(size_t endIndex)
{
    lqpCounter_++;

    // if solver is HPIPM, solve the full problem
    if (settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::HPIPM_SOLVER)
    {
        solveFullLQProblem();
    }
    else if (settings_.lqocp_solver == Settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER)
    {
        // if solver is GNRiccati, solve the first stage and get solution
        lqocProblem_->x_ = x_;
        lqocProblem_->u_ = u_ff_;
        lqocSolver_->setProblem(lqocProblem_);

        for (int i = endIndex; i >= 0; i--)
            lqocSolver_->solveSingleStage(i);

        lqocSolver_->computeStateAndControlUpdates();
    }
    else
        throw std::runtime_error("unknown solver type in finishSolveLQProblem()");
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::solveFullLQProblem()
{
    lqpCounter_++;

    lqocProblem_->x_ = x_;
    lqocProblem_->u_ = u_ff_;

    if (lqocProblem_->isBoxConstrained())
    {
        if (settings_.debugPrint)
            std::cout << "LQ Problem has box constraints. Configuring box constraints now. " << std::endl;

        lqocSolver_->configureBoxConstraints(lqocProblem_);
    }
    if (lqocProblem_->isGeneralConstrained())
    {
        if (settings_.debugPrint)
            std::cout << "LQ Problem has general constraints. Configuring general constraints now. " << std::endl;

        lqocSolver_->configureGeneralConstraints(lqocProblem_);
    }

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

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::retrieveLastLinearizedModel(StateMatrixArray& A,
    StateControlMatrixArray& B)
{
    A = lqocProblem_->A_;
    B = lqocProblem_->B_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::Policy_t&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSolution()
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


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::doFullStepUpdate()
{
    u_ff_ += lu_;
    x_ += lx_;

    alphaBest_ = 1.0;

    if (settings_.debugPrint || settings_.printSummary)
    {
        lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(lx_);
        lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(lu_);
    }
    else
    {
#ifdef MATLAB
        lx_norm_ = computeDiscreteArrayNorm<StateVectorArray, 2>(lx_);
        lu_norm_ = computeDiscreteArrayNorm<ControlVectorArray, 2>(lu_);
#endif
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logSummaryToMatlab(const std::string& fileName)
{
    summaryAllIterations_.logToMatlab(fileName);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const SummaryAllIterations<SCALAR>& NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSummary() const
{
    return summaryAllIterations_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getStateUpdates()
{
    lx_ = lqocSolver_->getStateUpdates();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getControlUpdates()
{
    lu_ = lqocSolver_->getControlUpdates();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getFeedback()
{
    if (settings_.closedLoopShooting)
        lqocSolver_->getFeedback(L_);
    else
        L_.setConstant(core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>::Zero());  // TODO should eventually go away
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::resetDefects()
{
    lqocProblem_->b_.setConstant(state_vector_t::Zero());
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::computeDefectsNorm()
{
    d_norm_ = computeDefectsNorm<1>(lqocProblem_->b_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
size_t& NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::iteration()
{
    return iteration_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::nominalRollout()
{
    bool success =
        rolloutSingleShot(settings_.nThreads, 0, u_ff_, x_, x_prev_, xShot_, *this->substepsX_, *this->substepsU_);
    x_prev_ = x_;
    u_ff_prev_ = u_ff_;
    firstRollout_ = false;
    return success;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::TimeArray&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getTimeArray()
{
    return t_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::isConfigured()
{
    return configured_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::isInitialized()
{
    return initialized_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getTotalDefect() const
{
    return d_norm_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::testConsistency()
{
    return true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::DynamicsPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getNonlinearSystemsInstances()
{
    return systems_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::DynamicsPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getNonlinearSystemsInstances() const
{
    return systems_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::LinearPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getLinearSystemsInstances()
{
    return linearSystems_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::LinearPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getLinearSystemsInstances() const
{
    return linearSystems_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::CostFunctionPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getCostFunctionInstances()
{
    return costFunctions_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::CostFunctionPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getCostFunctionInstances() const
{
    return costFunctions_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getBoxConstraintsInstances()
{
    return boxConstraints_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getBoxConstraintsInstances() const
{
    return boxConstraints_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getGeneralConstraintsInstances()
{
    return generalConstraints_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getGeneralConstraintsInstances() const
{
    return generalConstraints_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getTimeHorizon()
{
    return K_ * settings_.dt;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
int NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getNumSteps()
{
    return K_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
int NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getNumStepsPerShot()
{
    return settings_.K_shot;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const typename NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::Settings_t&
NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSettings() const
{
    return settings_;
}

}  // namespace optcon
}  // namespace ct


#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
