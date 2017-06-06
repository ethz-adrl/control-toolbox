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

#ifndef CT_OPTCON_DMS_CORE_DERIVATIVES_FULL_PWL_H_
#define CT_OPTCON_DMS_CORE_DERIVATIVES_FULL_PWL_H_

#include "DerivativeBase.h"

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      The derivative size
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template<size_t STATE_DIM, size_t CONTROL_DIM>
struct DerivativeFullPwlSize
{
	static const size_t SIZE =
			STATE_DIM + STATE_DIM*STATE_DIM + STATE_DIM*CONTROL_DIM
			+ STATE_DIM*CONTROL_DIM +
			STATE_DIM + CONTROL_DIM +
			CONTROL_DIM;
};


/**
 * @ingroup    DMS
 *
 * @brief      Implementation of the derivatives using costevaluator full and
 *             piecewise linear control input splining
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template<size_t STATE_DIM, size_t CONTROL_DIM>
class DerivativeFullPwl : 
public ct::core::System <DerivativeFullPwlSize<STATE_DIM, CONTROL_DIM>::SIZE>,
public DerivativeBase<STATE_DIM, CONTROL_DIM, DerivativeFullPwlSize<STATE_DIM, CONTROL_DIM>::SIZE >
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int derivativeSize_ = 	DerivativeFullPwlSize<STATE_DIM, CONTROL_DIM>::SIZE;

	typedef DerivativeBase<STATE_DIM, CONTROL_DIM, derivativeSize_> Base;

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

	typedef typename Base::derivative_vector_t derivative_vector_t;
	typedef typename Base::derivative_traj_t derivative_traj_t;

	DerivativeFullPwl() = delete;

	DerivativeFullPwl(
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

	virtual ~DerivativeFullPwl(){}

	virtual void retrieveStateTrajectories(
		time_array_t& timeTraj,
		state_vector_array_t& stateTraj,
		double& cost
		) override
	{}

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
			currdXdQi = devTrajectory_[i].segment(ind, STATE_DIM * CONTROL_DIM);
			dXdQip1Traj.push_back(Eigen::Map<state_control_matrix_t> (currdXdQi.data()));
			ind += STATE_DIM * CONTROL_DIM;					
		}
		costGradientSi = devTrajectory_.back().segment(ind, STATE_DIM);
		ind += STATE_DIM;
		costGradientQi = devTrajectory_.back().segment(ind, CONTROL_DIM);
		ind += CONTROL_DIM;
		costGradientQip1 = devTrajectory_.back().segment(ind, CONTROL_DIM);		
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
		Base::updateMatrices(t);
		Base::dUdQi_ = Base::controlSpliner_->splineDerivative_q_i(t, Base::shotNr_);
		Base::dUdQip1_ = Base::controlSpliner_->splineDerivative_q_iplus1(t, Base::shotNr_);	


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

		//dXdQip1
		currdXdQiVec = state.segment(count, STATE_DIM*CONTROL_DIM);
		Base::dXdQip1_ = Eigen::Map<state_control_matrix_t> (currdXdQiVec.data());
		count = Base::dxdqip1Derivative(derivative, count);

		//Cost Gradient
		Base::costFct_->setCurrentStateAndControl(Base::state_, Base::control_);
		Base::costStateDer_ = Base::costFct_->stateDerivativeIntermediate();
		Base::costControlDer_ = Base::costFct_->controlDerivativeIntermediate();
		count = Base::dLdsiDerivative(derivative, count);
		count = Base::dLdqiDerivative(derivative, count);
		count = Base::dLdqip1Derivative(derivative, count);
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
