/*
 * date created: 01.03.2016
 * author: mgiftthaler@ethz.ch
 *
 * */

// TODO: currently we first spline, then we distribute the spliner to different cost functions. Make different, so that we can call the stabiliser repeatedly?

#ifndef DMS_ILQG_STABILISER_HPP_
#define DMS_ILQG_STABILISER_HPP_

#include <ct/optcon/dms/stabilisation/StabDimensions.hpp>
#include <ct/optcon/ilqg/iLQGMP.hpp>
#include <ct/optcon/lqr/LQR.hpp>
#include <ct/optcon/lqr/FHDTLQR.hpp>

#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/systems/linear/LinearSystem.h>
#include <ct/core/systems/linear/SystemLinearizer.h>

#include <ct/core/control/StateFeedbackController.h>

#include <math.h>

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM>
class iLQG_Stabiliser : public StabiliserBase<STATE_DIM, CONTROL_DIM>
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


	iLQG_Stabiliser(
			std::shared_ptr<ct::optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM> > costFunction,
			stab_settings settings,
			std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem,
			std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem = nullptr
	) :
		BASE(costFunction, settings, controlledSystem, linearSystem)
	{
		size_t K_MAX = 5000;
		mp_settings_.nThreads = settings.nThreads_;
		line_search_settings_.active = true;

		for (size_t i=0; i<mp_settings_.nThreads+1; i++)
		{
			nonlinearSystems_mp_.push_back(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> (BASE::controlledSystem_->clone()));
			linearSystems_mp_.push_back(std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> (BASE::linearSystem_->clone()));
			costFunctionsTracking_.push_back(std::shared_ptr<optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM>>(costFunction->clone()));
			costFunctions_.push_back(std::static_pointer_cast<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>>(costFunctionsTracking_.back())); // todo: this pointer cast should not be required
		}
		// initialize ilqg solver
		ilqg_mp_ = std::shared_ptr<optcon::iLQGMP<STATE_DIM, CONTROL_DIM>> ( new
				optcon::iLQGMP<STATE_DIM, CONTROL_DIM> (nonlinearSystems_mp_, linearSystems_mp_, costFunctions_, K_MAX, mp_settings_));
	}

	virtual ~iLQG_Stabiliser(){}

	virtual	void updateNominalCostVectors(
		const state_vector_t& x_nom,
		const state_vector_t& x_des
		) override
	{
		for(auto costFunction : costFunctions_)
		{
			costFunction->updateReferenceState(x_nom);
			costFunction->updateFinalState(x_des);
		}		
	}


	bool stabilise(
			const state_vector_array_t& x_traj,
			const control_vector_array_t& u_traj,
			const time_array_t& t_traj,
			const state_matrix_t& Q_stab,
			const control_matrix_t& R_stab,
			const state_matrix_t& Q_final_stab,
			const double dt_sim,
			const state_vector_t& current_x) override
	{

		if(x_traj.size() != u_traj.size())
			throw std::runtime_error("ilqg stabilizer : x_traj.size() != u_traj.size()");
		if(x_traj.size() != t_traj.size())
			throw std::runtime_error("ilqg stabilizer : x_traj.size() != t_traj.size()");

		BASE::stateSpliner_->genSpline(x_traj, t_traj);
		BASE::controlSpliner_->genSpline(u_traj, t_traj);

		for(auto costFunction : costFunctionsTracking_)
			costFunction->updateTrajectories(x_traj, u_traj, t_traj);

		dt_sim_ = dt_sim;
		numSteps_ = round(t_traj.back() / dt_sim);

		auto start = std::chrono::system_clock::now();

		size_t K = numSteps_;
		control_feedback_array_t u0_fb(K);
		control_vector_array_t u0_ff(K);

		//-------------------TVLQR INITIALIZATION-------------------------------//

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

		std::shared_ptr<optcon::FHDTLQR<STATE_DIM, CONTROL_DIM>> fhdtlqr (new optcon::FHDTLQR<STATE_DIM, CONTROL_DIM> (costFunctions_.front()));

		bool performNumericalChecks = false;

		fhdtlqr->designController(x_interp, u_interp, A_traj, B_traj, dt_sim, u0_fb, performNumericalChecks);

		for (size_t i = 0; i< numSteps_; i++)
		{
			u0_ff[i] = u_interp[i] - u0_fb[i] * x_interp[i];
		}

		//-------------------TVLQR END-------------------------------//

		ilqg_settings_.nSteps = K;
		ilqg_settings_.dt = dt_sim_;
		ilqg_settings_.dt_sim = dt_sim_;
		ilqg_settings_.min_cost_improvement = 0.005;

		ilqg_mp_->initialize(current_x, u0_fb, u0_ff, ilqg_settings_);

		bool foundBetter_mp = true;
		size_t numIterations = 0;

		while (foundBetter_mp)
		{
			// solve
			foundBetter_mp = ilqg_mp_->runIteration(ilqg_settings_, line_search_settings_);
			numIterations++;
		}

		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		std::cout << "... ilQG terminated in " << elapsed.count() << " ms after "<< numIterations << "iterations. " << std::endl;

		return true;
	}

	virtual const state_vector_array_t retrieveStateRollout() const override
	{
		return ilqg_mp_->retrieveLastRollout();
	}

	virtual const control_vector_array_t retrieveControlRollout() const override
	{
		return ilqg_mp_->retrieveLastControlInput();
	}

	virtual const time_array_t retrieveTimeRollout() const override
	{
		return ilqg_mp_->retrieveLastTimeArray();
	}

	virtual const control_feedback_array_t retrieveFeedbackGains() const
	{
		return ilqg_mp_->retrieveFeedbackGains();
	}

	virtual const control_vector_array_t retrieveFeedforwardGains() const
	{
		return ilqg_mp_->retrieveLastFeedforwardControlInput();
	}


protected:

	double dt_sim_;
	size_t numSteps_;

	ilqg_settings_t ilqg_settings_;
	line_search_settings_t line_search_settings_;
	multi_processing_settings_t mp_settings_;

	std::vector<std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM> > > nonlinearSystems_mp_;
	std::vector<std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM> > > linearSystems_mp_;
	std::vector<std::shared_ptr<optcon::CostFunctionQuadraticTracking<STATE_DIM, CONTROL_DIM>>> costFunctionsTracking_;
	std::vector<std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>>> costFunctions_;

	std::shared_ptr<optcon::iLQGMP<STATE_DIM, CONTROL_DIM>> ilqg_mp_;

};

} // namespace optcon
} // namespace ct

#endif
