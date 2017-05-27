/*
 * MPC_iLQG.hpp
 *
 *  Created on: 23.06.2014
 *      Author: neunertm
 */

 // #include <ct/optcon/dms/mpc/MPC.hpp>

#include <boost/numeric/odeint.hpp>

#include <chrono>

using namespace boost::numeric;
using namespace std;
using namespace std::chrono;

//#define DEBUG_ILQG_MPC

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MPC_iLQG<STATE_DIM, CONTROL_DIM>::run(
	const state_vector_t& stateIn, 
	control_feedback_array_t& KOut,
	control_vector_array_t& uffOut
	)
{
	high_resolution_clock::time_point start, end;
	static microseconds lastDelayUs(0);

//	cout << "MPC 1: Starting MPC. "<<endl;

	microseconds delayUs(0);

	// if adaptive delay
	if (settingsMpc_.measureDelay_)
	{
		start = high_resolution_clock::now();

		delayUs = lastDelayUs;

	} else // fixed delay
	{
		delayUs = microseconds(static_cast<int>(settingsMpc_.fixedDelayUs_ + settingsMpc_.additionalDelayUs_));
	}

//	cout << "MPC 2: Calculated delay "<<delayUs.count()<<" us."<<endl;

	// find forward state by integration based on delay
	state_vector_t startState;
	// std::cout << "stateIn: " << stateIn << std::endl;
	// std::cout << "stateInV: " << stateInV << std::endl;
	if (delayUs > microseconds(0) && !firstRun_)
	{
		// cout << "MPC 2b: Delay detected. Will forward simulate."<<endl;
		// cout << "Delay: "<<delayUs.count()<<" us"<<endl;
		rollout(stateIn, previousK_, previousUff_, startState, delayUs.count());
		// startState = stateIn;
		// cout << "state diff: "<<stateIn - startState<<endl;
		// cout << "MPC 2b: Finished forward simulating."<<endl;
	} else
	{
		startState = stateIn;
	}

//	cout << "MPC 3: Will calculate control now."<<endl;
	calculateController(startState, previousK_, previousUff_, delayUs.count());
	firstRun_ = false;
	// calculateController(startState, previousK_, previousUff_, 0);
//	cout << "MPC 3: Finished calculating control."<<endl;

	if (settingsMpc_.measureDelay_)
	{
		lastDelayUs = duration_cast<microseconds>(high_resolution_clock::now() - start) + microseconds(settingsMpc_.additionalDelayUs_);
	}

	KOut = previousK_;
	uffOut = previousUff_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MPC_iLQG<STATE_DIM, CONTROL_DIM>::rollout(const state_vector_t& stateInConst, 
		const control_feedback_array_t& K, 
		const control_vector_array_t& u,
		state_vector_t& stateOut,
		size_t steps)
{
	state_vector_t stateIn = stateInConst;
	state_vector_array_t statesOut;
	size_t control_steps = round(steps / this->ilqg_settings_.dt / 1000 / 1000);
	statesOut.resize(control_steps+1);

//	std::cout << "simulating forward for " << steps << " steps." << std::endl;
//	std::cout << "with " << control_steps << " control steps." << std::endl;
//	std::cout << "with " << this->dt_dim_ << " s steps" << std::endl;
//	std::cout << "with " << dt_control_ << " s ilqg steps" << std::endl;

	std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> stateFeedbackCtrl (new
		ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> (u, K, ilqg_settings_.dt));
	dynamics_->setController(stateFeedbackCtrl);
	// do rollout of stabilized solution
	time_array_t t_temp;
	integrator_->integrate_n_steps(stateIn, 0.0, control_steps, ilqg_settings_.dt, statesOut, t_temp);
	stateOut = statesOut.back();
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MPC_iLQG<STATE_DIM, CONTROL_DIM>::calculateController(
		state_vector_t& startState,
		control_feedback_array_t& kOut,
		control_vector_array_t& uffOut,
		const double delay_Us
		)
{
	size_t steps = ilqg_settings_.nSteps;
//	size_t steps = round(settingsMpc_.timeHorizon_/ilqg_settings_.dt); // number of control steps we will request from ilqg without delays
	size_t stepsPassed = round(delay_Us / ilqg_settings_.dt / 1000.0 / 1000.0); //conversion from microseconds to seconds

	if (steps <= stepsPassed)
	{
		std::cout << "Warning! Delay in MPC tool large!" << std::endl;
		for (size_t i=0; i<kOut.size(); i++)
		{
			kOut[i].setZero();
			uffOut[i].setZero();
		}
		return;
	}

	size_t iLQGSteps = steps - stepsPassed;
	size_t stepsToBeFilled = iLQGSteps - stepsPassed;
	ilqg_settings_.nSteps = iLQGSteps; // number of control steps we will request from ilqg

#ifdef DEBUG_ILQG_MPC
	std::cout << "MPC iLQG: Will run iLQG now (" << steps << " steps, " << stepsPassed << " passed)." << std::endl;
	std::cout << "MPC-iLQG: Initializing iLQG" << std::endl;
	std::cout << "MPC-iLQG: Steps initialized: " << iLQGSteps << std::endl;
#endif

	control_feedback_array_t newK(iLQGSteps);
	control_vector_array_t newUff(iLQGSteps);

	if(settingsMpc_.coldStart_)
	{
		for(size_t i = 0; i < iLQGSteps; ++i)
		{
			newK[i] = K_init_[i];
			newUff[i] = u_ff_init_[i];	
		}
	}
	if(!settingsMpc_.coldStart_)
	{
		// Take as many elements from previous iteration as possible
		// It is not possible to take all the elements, due to the time shift (except for the first iteration)
		for(size_t i = 0; i < stepsToBeFilled; ++i)
		{
			newK[i] = kOut[i + stepsPassed];
			newUff[i] = uffOut[i + stepsPassed];
		}

		for(size_t i = stepsToBeFilled; i < iLQGSteps; ++i)
		{
			newK[i] = kOut.back();
			newUff[i] = uffOut.back();	
		}
	}

	ilqg_mp_->initialize(startState, newK, newUff, ilqg_settings_);


	bool better_solution = true;
	size_t i = 0;
	// TODO: ! if the initial controller is the best solution, nothing is written and x_traj/u_traj are wrong!
	while (better_solution && i < settingsMpc_.maxIterations_) {

#ifdef DEBUG_ILQG_MPC
		std::cout<<"MPC iLQG: Running iteration "<<i+1<<" of "<< settingsMpc_.maxIterations_ << std::endl;
#endif

		better_solution = ilqg_mp_->runIteration(ilqg_settings_, lineSearchSettings_);
		if (better_solution) {
			newK = ilqg_mp_->retrieveFeedbackGains();
			newUff = ilqg_mp_->retrieveLastFeedforwardControlInput();
			x_traj_ = ilqg_mp_->retrieveLastRollout();
			u_traj_ = ilqg_mp_->retrieveLastControlInput();

#ifdef DEBUG_ILQG_MPC
			std::cout << "MPC iLQG> Better solution was found" << std::endl;
			std::cout << "Trajectory size: " << x_traj_.size() << "resp. " << u_traj_.size() << std::endl;
#endif
		}

#ifdef DEBUG_ILQG_MPC
		std::cout<<"MPC iLQG: Completed iteration "<<i+1<<" of "<< settingsMpc_.maxIterations_ <<std::endl;
		std::cout << "bool is " << (int)better_solution << std::endl;
#endif

		++i;
	}

#ifdef DEBUG_ILQG_MPC
	std::cout << "MPC_iLQG> size new Controller: " << newUff.size() << std::endl;
#endif


	for(size_t i = 0; i < iLQGSteps; ++i)
	{
		kOut[i] = newK[i];
		uffOut[i] = newUff[i];
	}

	kOut.resize(iLQGSteps);
	uffOut.resize(iLQGSteps);

#ifdef DEBUG_ILQG_MPC
	std::cout << "MPC_iLQG> size returned Controller: " << uffOut.size() << std::endl;
	std::cout << "and trajectory of size: " << x_traj_.size() << "resp. " << u_traj_.size() << std::endl;
#endif
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
typename MpcDimensions<STATE_DIM, CONTROL_DIM>::state_vector_array_t MPC_iLQG<STATE_DIM, CONTROL_DIM>::getLatestStateTraj() {
	return x_traj_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MPC_iLQG<STATE_DIM, CONTROL_DIM>::getLatestInputTraj(control_vector_array_t& u_traj) {
	u_traj = u_traj_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void MPC_iLQG<STATE_DIM, CONTROL_DIM>::updateControlGains(
		const control_feedback_array_t& K_traj,
		const control_vector_array_t& u_ff) {
	K_init_ = K_traj;
	previousK_ = K_traj;
	u_ff_init_ = u_ff;
	previousUff_ = u_ff;
}
