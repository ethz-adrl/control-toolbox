/*
 * MPC_iLQG.hpp
 *
 *  Created on: 23.06.2014
 *      Author: neunertm
 */

#ifndef MPC_DMS_HPP_
#define MPC_DMS_HPP_

#include <ct/optcon/dms/dms_core/dms.hpp>
#include <ct/core/control/StateFeedbackController.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/core/types/trajectories/spliner/TrajectoryZohSpliner.hpp>
#include <ct/core/types/trajectories/spliner/TrajectoryLinearSpliner.hpp>

// #include <fstream>
// #include <cereal/archives/xml.hpp>
// #include <cereal/types/vector.hpp>
// #include <cereal/types/Eigen.hpp>

#include "MpcDimensions.hpp"
#include "MpcCommon.hpp"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class MpcDms
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef MpcDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;

	typedef typename DIMENSIONS::time_array_t time_array_t;


	MpcDms(
			std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM> > dynamics,
			std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem,
			std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> > costFunction,
			const state_vector_array_t& x_traj_init,
			const control_vector_array_t& u_traj_init,
			const state_vector_t& initState,
			const state_vector_t& terminalState,
			std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM>>> customConstraints,
			const dms_settings& settingsDms,
			const mpc_settings& settingsReplan
			) :
		dynamics_(dynamics),
		costFunction_(costFunction),
		initState_(initState),
		terminalState_(terminalState),
		previousX_(x_traj_init),
		previousU_(u_traj_init),
		lastTime_(0.0),
		settingsReplan_(settingsReplan),
		settingsDms_(settingsDms),
		timeHorizon_(settingsDms.T_)
	{
		std::cout << "Entered MPC_DMS constructor" << std::endl;

		integrator_  = std::shared_ptr<core::IntegratorRK4<STATE_DIM>> (
				new core::IntegratorRK4<STATE_DIM> (dynamics_));

		stateSpliner_ = std::shared_ptr<core::TrajectoryLinearSpliner<state_vector_t>> (
				new core::TrajectoryLinearSpliner<state_vector_t> ());

		settingsDms_.checkDerivatives_ = false;

		switch(settingsDms_.splineType_)
		{
			case ZERO_ORDER_HOLD:
				controlSpliner_ = std::shared_ptr<core::TrajectoryZohSpliner<control_vector_t>> (
					new core::TrajectoryZohSpliner<control_vector_t> ());
				break;
			case PIECEWISE_LINEAR:
				controlSpliner_ = std::shared_ptr<core::TrajectoryLinearSpliner<control_vector_t>> (
					new core::TrajectoryLinearSpliner<control_vector_t> ());
				break;
			default:
				std::cout << "Unknown Spline Type" << std::endl;
		}


		my_dms_ = std::shared_ptr<dms<STATE_DIM, CONTROL_DIM>>(
				 new dms<STATE_DIM, CONTROL_DIM>(dynamics, costFunction, settingsDms_, linearSystem));

		my_dms_->initialize(initState, terminalState, x_traj_init, u_traj_init, customConstraints);


		// Can try this
		// my_dms_->fastIntegration(true);

		// We are using SNOPT for warmstart solving
		switch(settingsReplan_.solverType_)
		{
			case IPOPT:
				solverReplanning_ = std::shared_ptr<IpoptDms<STATE_DIM, CONTROL_DIM>> 
					(new IpoptDms<STATE_DIM, CONTROL_DIM>(my_dms_, settingsDms_));
				break;
			case SNOPT:
				solverReplanning_ = std::shared_ptr<SnoptADms<STATE_DIM, CONTROL_DIM>> 
					(new SnoptADms<STATE_DIM, CONTROL_DIM>(my_dms_, settingsDms_));
				break;
			default:
				std::cout << "Unknown NLP solver" << std::endl;

		}

		std::cout << "Initializing inside MPC" << std::endl;
		
		solverReplanning_->Initialize_NLP();

		std::cout << "INitializing MPC done" << std::endl;
		solverReplanning_->PrepareWarmStart(settingsReplan_.maxIterations_);



		i = 0;
		std::cout << "Finished MPC_DMS constructor" << std::endl;	
	}

	void replanTrajectory(
		const state_vector_t& currState,
		const core::Time currTime);

	void getAnimation(
			const double dtInt,
			state_vector_array_t& x_animation,
			control_vector_array_t& u_animation,
			time_array_t& t_animation
		);

	void getSolution(
		state_vector_array_t& x_solution, 
		control_vector_array_t& u_solution, 
		time_array_t& t_solution
		);

	void setInitGuess(
			const state_vector_array_t& x_traj_init,
			const control_vector_array_t& u_traj_init,
			const time_array_t t_traj_init	
		)
	{
		previousX_ = x_traj_init;
		previousU_ = u_traj_init;
		previousT_ = t_traj_init;
	}

	void updateInitialState(const state_vector_t& x_0)
	{
		my_dms_->updateInitialState(x_0);
		initState_ = x_0;
	}

	void updateDesiredState(const state_vector_t& x_des)
	{
		my_dms_->updateDesiredState(x_des);
		terminalState_ = x_des;
	}

	void resetTimeHorizon(const double initialDmsHorizon)
	{
		my_dms_->updateTimeHorizon(initialDmsHorizon);
		timeHorizon_ = initialDmsHorizon;
		lastTime_ = 0.0;
	}

	void setControlTrajectories(
		const control_vector_array_t& u_ff,
		const control_feedback_array_t& u_fb)
	{
		u_ff_traj_ = u_ff;
		K_traj_ = u_fb;
	}

private:

	void rollout(const state_vector_t& stateIn, state_vector_t& stateOut, const double delay);
	void shiftInitGuess(const state_vector_t& startState, const core::Time currTime);
	void updateInitGuess(const state_vector_t& startState, const core::Time currTime);
	void exportTrajectories();

	std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM>> dynamics_;
	std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFunction_;

	state_vector_t initState_;
	state_vector_t terminalState_;
	state_vector_array_t previousX_;
	control_vector_array_t previousU_;
	// Time stamp of the start of the last replanning iteration
	double lastTime_;

	mpc_settings settingsReplan_;	
	dms_settings settingsDms_;
	double timeHorizon_;

	std::shared_ptr<dms<STATE_DIM, CONTROL_DIM>> my_dms_;
	std::shared_ptr<DmsSolverBase<STATE_DIM, CONTROL_DIM>> solverReplanning_;

	// Init init guesses from previous iteration
	std::shared_ptr<core::IntegratorBase<STATE_DIM>> integrator_;
	control_feedback_array_t K_traj_;
	control_vector_array_t u_ff_traj_;

	std::shared_ptr<core::TrajectorySplinerBase<state_vector_t>> stateSpliner_;
	std::shared_ptr<core::TrajectorySplinerBase<control_vector_t>> controlSpliner_;

	//used for xml exporting
	size_t i;

	time_array_t previousT_; // not sure if needed though

	state_vector_array_t xInitguess_;
	control_vector_array_t uInitguess_;
	time_array_t newT_;

};

#include "implementation/MPC_DMS.hpp"

} // namespace optcon
} // namespace ct

#endif /* MPC_DMS_HPP_ */
