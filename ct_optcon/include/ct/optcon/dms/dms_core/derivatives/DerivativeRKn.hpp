#ifndef DERIVATIVES_VECTORIZED_DERIVATIVE_RKN1_HPP_
#define DERIVATIVES_VECTORIZED_DERIVATIVE_RKN1_HPP_

#include <ct/core/core.h>
#include <ct/optcon/dms/dms_core/RKnDerivatives.hpp>

#include "DerivativeBase.hpp"

namespace ct {
namespace optcon {

template<size_t STATE_DIM, size_t CONTROL_DIM>
class DerivativeRKn : public DerivativeBase <STATE_DIM, CONTROL_DIM, STATE_DIM>,
public ct::core::System <STATE_DIM>
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	const static size_t derivativeSize_ = STATE_DIM;

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef DerivativeBase<STATE_DIM, CONTROL_DIM, derivativeSize_> Base;

	typedef typename Base::derivative_vector_t derivative_vector_t;
	typedef typename Base::derivative_traj_t derivative_traj_t;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t  control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	DerivativeRKn() = delete;

	DerivativeRKn(
			std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem,
			std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem,
			std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct,
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
			std::shared_ptr<TimeGrid> timeGrid,
			size_t shotNr,
			DmsSettings settings
	)
	:
		Base(controlledSystem, linearSystem, costFct, w, controlSpliner, timeGrid, shotNr, settings),
		rknDerivatives_(controlSpliner, shotNr, settings)
	{}

	virtual ~DerivativeRKn(){}

	virtual void retrieveStateTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			double& cost
	) override {}

	virtual void retrieveTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			state_matrix_array_t& dXdSiTraj,
			state_control_matrix_array_t& dXdQiTraj,
			state_control_matrix_array_t& dXdQip1Traj,
			state_vector_array_t& dXdHiTraj,
			state_vector_t& costGradientSi,
			control_vector_t& costGradientQi,
			control_vector_t& costGradientQip1,
			double& costGradientHi

	) override {
		timeTraj.clear();
		stateTraj.clear();
		dXdSiTraj.clear();
		dXdQiTraj.clear();
		dXdQip1Traj.clear();
		dXdHiTraj.clear();
		timeTraj = timeTraj_;
		stateTraj = stateTraj_;
		rknDerivatives_.getdXdSiTraj(dXdSiTraj);
		rknDerivatives_.getdXdQiTraj(dXdQiTraj);
		rknDerivatives_.getdXdQip1Traj(dXdQip1Traj);
		rknDerivatives_.getdXdHiTraj(dXdHiTraj);
		costGradientHi = 0.0;
	}

	virtual void initForIntegration() override {

		stateTrajRkn_ = std::shared_ptr<state_vector_array_t> (new state_vector_array_t ());
		controlTrajRkn_ = std::shared_ptr<control_vector_array_t> (new control_vector_array_t ());
		timeTrajRkn_ = std::shared_ptr<time_array_t> (new time_array_t ());

		Base::system_->startLoggingStates(stateTrajRkn_); //note that x_log will be one element shorter than x_history, when using euler integration
		Base::system_->startLoggingControls(controlTrajRkn_);
		Base::system_->startLoggingTimes(timeTrajRkn_);  //note that t_log will be one element shorter than t_history, when using euler integration

	}

	virtual void wrapUpIntegration() override {
		Base::system_->stopLoggingStates();
		Base::system_->stopLoggingControls();
		Base::system_->stopLoggingTimes();
		timeTrajRkn_->push_back(timeTraj_.back());

		ATrajRkn_ =std::shared_ptr<state_matrix_array_t> (new state_matrix_array_t ());
		BTrajRkn_ =std::shared_ptr<state_control_matrix_array_t> (new state_control_matrix_array_t ());

		computeHistoryOfLinMatrices(stateTrajRkn_, controlTrajRkn_, timeTrajRkn_, ATrajRkn_, BTrajRkn_);

		rknDerivatives_.setLogs(stateTrajRkn_, controlTrajRkn_, timeTrajRkn_, ATrajRkn_, BTrajRkn_);
		rknDerivatives_.compute_dXdSi();
		rknDerivatives_.compute_dXdQi();
		if(Base::settings_.splineType_ == DmsSettings::PIECEWISE_LINEAR)
			rknDerivatives_.compute_dXdQip1();
		if(Base::settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
			rknDerivatives_.compute_dXdHi();
	}


	virtual void computeDynamics(
			const state_vector_t& state,
			const ct::core::Time& t,
			state_vector_t& derivative
	) override {

		this->system_->computeDynamics(state, t, derivative);
	}


	const state_vector_t getInitState() {return Base::w_->getOptimizedState(Base::shotNr_);}
	state_vector_array_t& stateTrajectory() {return stateTraj_;}
	time_array_t& timeTrajectory() {return timeTraj_;}


private:

	RKnDerivatives<STATE_DIM, CONTROL_DIM> rknDerivatives_;
	std::shared_ptr<state_vector_array_t> stateTrajRkn_;
	std::shared_ptr<control_vector_array_t> controlTrajRkn_;
	std::shared_ptr<time_array_t> timeTrajRkn_;
	std::shared_ptr<ct::core::IntegratorBase<STATE_DIM>> integrator_;

	std::shared_ptr<state_matrix_array_t> ATrajRkn_;
	std::shared_ptr<state_control_matrix_array_t> BTrajRkn_;

	state_vector_array_t stateTraj_;
	time_array_t timeTraj_;

	void computeHistoryOfLinMatrices(
			const std::shared_ptr<state_vector_array_t>x_log,
			const std::shared_ptr<control_vector_array_t> u_log,
			const std::shared_ptr<time_array_t> t_log,
			std::shared_ptr<state_matrix_array_t> A_log,
			std::shared_ptr<state_control_matrix_array_t> B_log)
	{
		size_t hist_length = x_log->size();

		assert(x_log->size() == u_log->size());

		A_log->resize(hist_length); //todo: avoid resize operations for faster running. E.g. limit the number of integration steps and make this member variable
		B_log->resize(hist_length);

		for(size_t i = 0; i< hist_length; ++i)
		{
			(*A_log)[i] =  Base::linearSystem_-> getDerivativeState((*x_log)[i], (*u_log)[i]);
			(*B_log)[i] =  Base::linearSystem_-> getDerivativeControl((*x_log)[i], (*u_log)[i]);
		}
	}	

};

} // namespace optcon
} // namespace ct

#endif //DERIVATIVES_VECTORIZED_DERIVATIVES_ZOH_SIMPLE_HPP_
