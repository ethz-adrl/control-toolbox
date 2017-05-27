/* A simple time-varying lqr controller without feedforward.
 * Designs only feedback gains.
 *
 * date created: 01.03.2016
 * author: mgiftthaler@ethz.ch
 *
 *
 * TODO: spline control inputs as splined in dms, instead of splining them as
 * */

#ifndef DMS_TVLQR_STABILISER_HPP_
#define DMS_TVLQR_STABILISER_HPP_

#include <math.h>

#include <ct/optcon/dms/stabilisation/StabDimensions.hpp>
#include <ct/optcon/dms/stabilisation/StabiliserBase.hpp>

#include <ct/core/types/StateVector.h>
#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/systems/linear/SystemLinearizer.h>
#include <ct/core/types/trajectories/FeedbackArray.h>

#include <ct/core/control/StateFeedbackController.h>

#include <ct/optcon/lqr/FHDTLQR.hpp>

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM>
class TVLQR_Stabiliser : public StabiliserBase<STATE_DIM, CONTROL_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef StabDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef StabiliserBase<STATE_DIM, CONTROL_DIM> BASE;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;
	typedef typename DIMENSIONS::control_feedback_t control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;


	TVLQR_Stabiliser(
			std::shared_ptr<ct::optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM> > costFunction,
			stab_settings settings,
			std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem,
			std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem = nullptr
	) :
		BASE(costFunction, settings, controlledSystem, linearSystem)
	{}

	virtual	void updateNominalCostVectors(
		const state_vector_t& x_nom,
		const state_vector_t& x_des
		) override
	{
		BASE::costFunction_->updateReferenceState(x_nom);
		BASE::costFunction_->updateFinalState(x_des);
	}

	virtual bool stabilise(
			const state_vector_array_t& x_traj,
			const control_vector_array_t& u_traj,
			const time_array_t& t_traj,
			const state_matrix_t& Q_stab,
			const control_matrix_t& R_stab,
			const state_matrix_t& Q_final_stab,
			const double dt_sim,
			const state_vector_t& current_x
			) override
	{
		assert(x_traj.size() == u_traj.size());
		assert(x_traj.size() == t_traj.size());

		BASE::stateSpliner_->genSpline(x_traj, t_traj);
		BASE::controlSpliner_->genSpline(u_traj, t_traj);

		BASE::costFunction_->updateTrajectories(x_traj, u_traj, t_traj);

		dt_sim_ = dt_sim;
		numSteps_ = round(t_traj.back() / dt_sim);

		// size_t i = 0;
		bool foundSolution = true;

		auto start = std::chrono::system_clock::now();

		state_matrix_array_t A_traj;
		state_control_matrix_array_t B_traj;
		state_vector_array_t x_interp;
		control_vector_array_t u_interp;

		for (size_t i = 0; i< numSteps_; i++)
		{
			double time = i * dt_sim;
			state_vector_t state = BASE::stateSpliner_->evalSpline(time);
			control_vector_t control = BASE::controlSpliner_->evalSpline(time);

			// discrete time linearizations
			state_matrix_t A = state_matrix_t::Identity() + dt_sim_ * BASE::linearSystem_->getDerivativeState( state, control, time );
			state_control_matrix_t B = dt_sim_ * BASE::linearSystem_->getDerivativeControl( state, control, time);

			A_traj.push_back(A);
			B_traj.push_back(B);
			x_interp.push_back(state);
			u_interp.push_back(control);
		}

		x_interp.push_back(BASE::stateSpliner_->evalSpline(numSteps_*dt_sim));

		assert(x_interp.back() == x_traj.back());

		std::shared_ptr<optcon::FHDTLQR<STATE_DIM, CONTROL_DIM>> fhdtlqr (new optcon::FHDTLQR<STATE_DIM, CONTROL_DIM> (BASE::costFunction_));

		bool performNumericalChecks = false;

		fhdtlqr->designController(x_interp, u_interp, A_traj, B_traj, dt_sim, BASE::K_traj_, performNumericalChecks);


		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds> (end - start);


		if(foundSolution)
			std::cout << "... found stabilizing FHDTLQR controller in " << elapsed.count() << " ms. " << std::endl;
		else
			std::cout << "... failed to find stabilizing FHDTLQR controller!" << std::endl;

		this->calcFeedforwardInput();		

		return foundSolution;
		}

	virtual const control_feedback_array_t retrieveFeedbackGains() const
	{
		return	BASE::K_traj_;
	}

	virtual const control_vector_array_t retrieveFeedforwardGains() const 
	{
		return BASE::u_ff_;
	}	




	void getControlVectorArrayStabilized(control_vector_array_t& u_stabilized)
	{
		u_stabilized = u_stabilized_;
	}


	virtual const state_vector_array_t retrieveStateRollout() const override
	{
		state_vector_array_t x_rollout;
		x_rollout.clear();
		double startTime = 0.0;
		ct::core::StateVector<STATE_DIM> x_start = BASE::stateSpliner_->evalSpline( startTime );
		assert(BASE::K_traj_.size() == numSteps_);

		ct::core::ControlVectorArray<CONTROL_DIM> uff_traj_mod;

		for (size_t i = 0; i< numSteps_; i++)
		{
			double time = dt_sim_ * i;
			ct::core::ControlVector<CONTROL_DIM> uff = BASE::controlSpliner_-> evalSpline(time) - BASE::K_traj_[i] * BASE::stateSpliner_->evalSpline(time) ;
			uff_traj_mod.push_back(uff);
		}

		std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> stateFeedbackCtrl (new
				ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM> (uff_traj_mod, BASE::K_traj_, dt_sim_));

		BASE::controlledSystem_->setController(stateFeedbackCtrl);

		// do rollout of stabilized solution
		time_array_t t_temp;
		BASE::integrator_->integrate_n_steps(x_start, startTime, numSteps_, dt_sim_, x_rollout, t_temp);

		return x_rollout;	
	}

	virtual const control_vector_array_t retrieveControlRollout() const override
	{
		control_vector_array_t u_rollout;
		for (size_t i = 0; i< numSteps_; i++)
			u_rollout.push_back(BASE::controlSpliner_->evalSpline(i * dt_sim_));

		return u_rollout;
	}

	virtual const time_array_t retrieveTimeRollout() const override
	{
		time_array_t t_rollout;
		for(size_t i = 0; i < numSteps_; ++i)
			t_rollout.push_back(i * dt_sim_);

		return t_rollout;
	}


private:

	double dt_sim_;
	size_t numSteps_;

	// temporary
	control_vector_array_t u_stabilized_;
	state_vector_array_t x_stabilized_;

	void calcFeedforwardInput()
	{
		assert(BASE::K_traj_.size() == numSteps_);

		for (size_t i = 0; i< numSteps_; i++)
		{
			double time = dt_sim_ * i;
			// BASE::u_ff_.push_back(BASE::controlSpliner_->evalSpline(time));
			BASE::u_ff_.push_back(BASE::controlSpliner_->evalSpline(time) - BASE::K_traj_[i] * BASE::stateSpliner_->evalSpline(time));
		}
	}

};

} // namespace optcon
} // namespace ct

#endif
