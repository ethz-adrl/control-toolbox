#ifndef DERIVATIVES_VECTORIZED_DERIVATIVES_STATE_COST_HPP_
#define DERIVATIVES_VECTORIZED_DERIVATIVES_STATE_COST_HPP_

#include "DerivativeBase.h"

namespace ct {
namespace optcon {


template<size_t STATE_DIM, size_t CONTROL_DIM>
class DerivativeStateCost : 
public ct::core::System <STATE_DIM + 1>, 
public DerivativeBase <STATE_DIM, CONTROL_DIM, STATE_DIM + 1>{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DerivativeBase<STATE_DIM, CONTROL_DIM, STATE_DIM + 1> Base;

	static const int derivativeSize_ = STATE_DIM + 1;

	typedef typename Base::derivative_vector_t derivative_vector_t;
	typedef typename Base::derivative_traj_t derivative_traj_t;

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
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

	DerivativeStateCost() = delete;

	DerivativeStateCost(
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
		Base(controlledSystem, linearSystem, costFct, w, controlSpliner, timeGrid, shotNr, settings)
	{}

	virtual ~DerivativeStateCost(){}

	virtual void retrieveStateTrajectories(
		time_array_t& timeTraj,
		state_vector_array_t& stateTraj,
		double& cost
		) override
	{
		timeTraj = timeTraj_;
		stateTraj.clear();
		for(size_t i = 0; i<devTrajectory_.size(); ++i)
		{
			stateTraj.push_back(devTrajectory_[i].segment(0, STATE_DIM));
		}

		cost = devTrajectory_.back()(STATE_DIM,0);
	}

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
	) override
	{
		timeTraj = timeTraj_;
		stateTraj.clear();
		dXdSiTraj.clear();
		dXdQiTraj.clear();
		dXdQip1Traj.clear();
		dXdHiTraj.clear();
		costGradientSi.setZero();
		costGradientQi.setZero();
		costGradientQip1.setZero();
		costGradientHi = 0.0;
	}

	const derivative_vector_t getInitState() {return devStart_;}
	derivative_traj_t& stateTrajectory() {return devTrajectory_;}
	time_array_t& timeTrajectory() {return timeTraj_;}


	private:

	derivative_vector_t devStart_;
	derivative_traj_t devTrajectory_;
	time_array_t timeTraj_;
	std::shared_ptr<ct::core::IntegratorBase<derivativeSize_>> integrator_;


	virtual void computeDynamics(
			const derivative_vector_t& state,
			const ct::core::Time& t,
			derivative_vector_t& derivative		
		) override	
	{
		size_t count = 0;

		Base::state_ = state.segment(count, STATE_DIM);
		Base::control_ = Base::controlSpliner_->evalSpline(t, Base::shotNr_);
		Base::costFct_->setCurrentStateAndControl(Base::state_, Base::control_);

		//State 
		count = Base::stateDerivative(t, derivative, count);

		//Cost
		count = Base::dLDerivative(derivative, count);
	}

	public:
		virtual void initForIntegration() override
	{
		devStart_.setZero();
		devStart_.segment(0, STATE_DIM) = Base::w_->getOptimizedState(Base::shotNr_);
	}

		virtual void wrapUpIntegration() override {}


};

} // namespace optcon
} // namespace ct

#endif //DERIVATIVES_VECTORIZED_DERIVATIVES_STATE_HPP_
