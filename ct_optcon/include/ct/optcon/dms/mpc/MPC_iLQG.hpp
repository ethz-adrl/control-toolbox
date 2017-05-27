/*
 * MPC_iLQG.hpp
 *
 *  Created on: 23.06.2014
 *      Author: neunertm
 */

#ifndef MPC_ILQG_HPP_
#define MPC_ILQG_HPP_

// #define DEBUG

#include <ct/optcon/ilqg/iLQG.hpp>
#include <ct/optcon/ilqg/iLQGMP.hpp>
#include "../../ilqg/iLQGSettings.hpp"
#include "MpcDimensions.hpp"
#include "MpcCommon.hpp"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class MPC_iLQG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// typedef MPC<STATE_DIM, CONTROL_DIM> BASE;

	typedef MpcDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;


	MPC_iLQG(
			std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM> > dynamics,
			std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM> > linearSystem,
			std::shared_ptr<ct::optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM> > costFunction,
			const mpc_settings& mpcSettings) 
	:
		settingsMpc_(mpcSettings),
		dynamics_(dynamics)
	{
		std::cout << "Entered MPC_ILQG constructor" << std::endl;
		std::cout << "Number of ILQG Threads: " << settingsMpc_.nThreads_ << std::endl;

		ilqg_settings_.nSteps = 1000;
		ilqg_settings_.dt = settingsMpc_.dt_control_;
		ilqg_settings_.dt_sim = settingsMpc_.dt_control_;

		assert(ilqg_settings_.dt > 0.0);
		assert(ilqg_settings_.dt_sim > 0.0);

		lineSearchSettings_.active = true;

		mp_settings_.nThreads = settingsMpc_.nThreads_;

		for (size_t i=0; i<mp_settings_.nThreads+1; i++)
		{
			nonlinearSystems_mp_.push_back(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> (dynamics_->clone()));
			linearSystems_mp_.push_back(std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> (linearSystem->clone()));
			costFunctionsTracking_.push_back(std::shared_ptr<ct::optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM>> (costFunction->clone()));
			costFunctions_mp_.push_back(std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> (costFunctionsTracking_.back()));
		}

		// initialize ilqg solver
		ilqg_mp_ = std::shared_ptr<ct::optcon::iLQGMP<STATE_DIM, CONTROL_DIM>> 
				(new ct::optcon::iLQGMP<STATE_DIM, CONTROL_DIM> (nonlinearSystems_mp_, linearSystems_mp_, costFunctions_mp_, ilqg_settings_.nSteps, mp_settings_));

		integrator_  = std::shared_ptr<ct::core::IntegratorRK4<STATE_DIM>> (
				new ct::core::IntegratorRK4<STATE_DIM> (dynamics_));

		// initialize trajectories
		x_traj_.resize(1);
		x_traj_[0].setZero();
		u_traj_.resize(1);
		u_traj_[0].setZero();
	}

	void run(
		const state_vector_t& stateIn,	
		control_feedback_array_t& KOut,	
		control_vector_array_t& uffOut
		);

	mpc_settings& settings() { return settingsMpc_; }

	void updateNominalTrajectories(
		const state_vector_array_t& xTraj,
		const control_vector_array_t& uTraj,
		const time_array_t& tTraj)
	{
		for(auto costFunction : costFunctionsTracking_)
			costFunction->updateTrajectories(xTraj, uTraj, tTraj);			
	}

	void rollout(
		const state_vector_t& stateInConst, 
		const control_feedback_array_t& K, 
		const control_vector_array_t& u,
		state_vector_t& stateOut,
		size_t steps);

	state_vector_array_t getLatestStateTraj();
	void getLatestInputTraj(control_vector_array_t& u_traj);

	void updateControlGains(
		const control_feedback_array_t& K_traj,
		const control_vector_array_t& u_ff);

	void setLineSearchSettings(const line_search_settings_t& settings) {
		lineSearchSettings_ = settings;
	}

	void updateTimeHorizon(const double& T){
		ilqg_settings_.nSteps = std::max(1,(int) ceil(T / settingsMpc_.dt_control_));
		std::cout << "setting steps N = " << ilqg_settings_.nSteps << std::endl;
	}

private:
	void calculateController(
			state_vector_t& startState,
			control_feedback_array_t& kOut,
			control_vector_array_t& uffOut,
			const double delay
		);

	std::shared_ptr<ct::optcon::iLQGMP<STATE_DIM, CONTROL_DIM>> ilqg_mp_;
	std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> dynamics_;
	control_feedback_array_t K_init_;
	control_vector_array_t u_ff_init_;

	control_feedback_array_t previousK_;
	control_vector_array_t previousUff_;

	state_vector_array_t x_traj_;
	control_vector_array_t u_traj_;

	line_search_settings_t lineSearchSettings_; /*< Settings for line search */
	multi_processing_settings_t mp_settings_;
	ilqg_settings_t ilqg_settings_;
	mpc_settings settingsMpc_;

	bool firstRun_;

	std::vector<std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM> > > nonlinearSystems_mp_;
	std::vector<std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM> > > linearSystems_mp_;
	std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> > > costFunctions_mp_;
	std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM> > > costFunctionsTracking_;
	std::shared_ptr<ct::core::IntegratorBase<STATE_DIM>> integrator_;

};

#include "implementation/MPC_iLQG.hpp"

} // namespace optcon
} // namespace ct

#endif /* MPC_ILQG_HPP_ */
