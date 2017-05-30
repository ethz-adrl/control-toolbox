#ifndef DERIVATIVES_VECTORIZED_DERIVATIVES_FULL_HPP_
#define DERIVATIVES_VECTORIZED_DERIVATIVES_FULL_HPP_

#include "DerivativeBase.hpp"

namespace ct {
namespace optcon {

template<size_t STATE_DIM, size_t CONTROL_DIM>
struct DerivativeFullZohSize
{
	static const size_t SIZE =STATE_DIM + STATE_DIM*STATE_DIM + STATE_DIM*CONTROL_DIM + STATE_DIM + CONTROL_DIM;
};


template<size_t STATE_DIM, size_t CONTROL_DIM>
class DerivativeFullZoh : 
public ct::core::System <DerivativeFullZohSize<STATE_DIM, CONTROL_DIM>::SIZE>,
public DerivativeBase<STATE_DIM, CONTROL_DIM, DerivativeFullZohSize<STATE_DIM, CONTROL_DIM>::SIZE>{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int derivativeSize_ = 	DerivativeFullZohSize<STATE_DIM, CONTROL_DIM>::SIZE;

	typedef DerivativeBase<STATE_DIM, CONTROL_DIM, derivativeSize_> Base;


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

	DerivativeFullZoh() = delete;

	DerivativeFullZoh(
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

	virtual ~DerivativeFullZoh(){}

	virtual void retrieveStateTrajectories(
		time_array_t& timeTraj,
		state_vector_array_t& stateTraj,
		double& cost
		) override
	{
		
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
		size_t ind = 0;
		for(size_t i = 0; i<devTrajectory_.size(); ++i)
		{
			ind = 0;
			stateTraj.push_back(devTrajectory_[i].segment(ind, STATE_DIM));
			ind += STATE_DIM;
			Eigen::Matrix<double, STATE_DIM*STATE_DIM, 1> currdXdSi = devTrajectory_[i].segment(ind, STATE_DIM*STATE_DIM);
			dXdSiTraj.push_back(Eigen::Map<state_matrix_t> (currdXdSi.data() ) );
			ind += STATE_DIM*STATE_DIM;
			Eigen::Matrix<double, STATE_DIM*CONTROL_DIM, 1> currdXdQi = devTrajectory_[i].segment(ind, STATE_DIM*CONTROL_DIM);
			dXdQiTraj.push_back(Eigen::Map<state_control_matrix_t> (currdXdQi.data() ) );
			ind += STATE_DIM*CONTROL_DIM;						
		}
		costGradientSi = devTrajectory_.back().segment(ind, STATE_DIM);
		ind += STATE_DIM;
		costGradientQi = devTrajectory_.back().segment(ind, CONTROL_DIM);
	}

	const derivative_vector_t getInitState() override {return devStart_;}
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
		Base::updateMatrices(t);
		Base::dUdQi_ = Base::controlSpliner_->splineDerivative_q_i(t, Base::shotNr_);

		//State 
		count = Base::stateDerivative(t, derivative, count);

		//dXdSi
		Eigen::Matrix<double, STATE_DIM*STATE_DIM, 1> currdXdSiVec = state.segment(count, STATE_DIM*STATE_DIM);
		Base::dXdSi_ = Eigen::Map<state_matrix_t> (currdXdSiVec.data());
		count = Base::dxdsiDerivative(derivative, count);

		//dXdQi
		Eigen::Matrix<double, STATE_DIM*CONTROL_DIM, 1> currdXdQiVec = state.segment(count, STATE_DIM*CONTROL_DIM);
		Base::dXdQi_ = Eigen::Map<state_control_matrix_t> (currdXdQiVec.data());
		count = Base::dxdqiDerivative(derivative, count);

		//Cost Gradient
		Base::costFct_->setCurrentStateAndControl(Base::state_, Base::control_);
		Base::costStateDer_ = Base::costFct_->stateDerivativeIntermediate();
		Base::costControlDer_ = Base::costFct_->controlDerivativeIntermediate();
		count = Base::dLdsiDerivative(derivative, count);
		count = Base::dLdqiDerivative(derivative, count);
	}

public:
	virtual void initForIntegration() override
	{
		devStart_.setZero();
		size_t count = 0;
		devStart_.segment(count, STATE_DIM) = Base::w_->getOptimizedState(Base::shotNr_);
		count += STATE_DIM;
		Eigen::Matrix<double, STATE_DIM, STATE_DIM> dXdSiStart;
		dXdSiStart.setIdentity();
		devStart_.segment(count, STATE_DIM*STATE_DIM) = Eigen::Map<Eigen::VectorXd> (dXdSiStart.data(), STATE_DIM*STATE_DIM);
	}	

	virtual void wrapUpIntegration() override {}


};

} // namespace optcon
} // namespace ct

#endif //DERIVATIVES_VECTORIZED_DERIVATIVES_FULL_HPP_
