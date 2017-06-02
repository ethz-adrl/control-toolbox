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

#ifndef CT_OPTCON_DMS_CORE_SHOTINTEGRATOR_H_
#define CT_OPTCON_DMS_CORE_SHOTINTEGRATOR_H_


#include "ShotIntegratorBase.h"
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

namespace ct{
namespace optcon{

/**
 * @brief      The implementation of the shotintegrator
 *
 * @tparam     DerivativeT  The type of derivative which will be integrated
 *                          depending the the dms settings
 */
template<class DerivativeT>
class ShotIntegrator : public ShotIntegratorBase<DerivativeT::STATE_D, DerivativeT::CONTROL_D>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef ShotIntegratorBase<DerivativeT::STATE_D, DerivativeT::CONTROL_D> Base;
	using IntegratorBase = ct::core::IntegratorBase<DerivativeT::derivativeSize_>;
	using IntegratorEuler =  ct::core::IntegratorEuler<DerivativeT::derivativeSize_>;
	using IntegratorRK4 = ct::core::IntegratorRK4<DerivativeT::derivativeSize_>;
	using IntegratorRK5Var = ct::core::IntegratorRK5Variable<DerivativeT::derivativeSize_>;

	typedef std::shared_ptr<ct::core::EventHandler<DerivativeT::derivativeSize_>> EventHandlerPtr;

	using EventHandlePtrVec = std::vector<EventHandlerPtr, Eigen::aligned_allocator<EventHandlerPtr>>;

	typedef typename Base::state_vector_t  state_vector_t;
	typedef typename Base::control_vector_t control_vector_t;
	typedef typename Base::state_vector_array_t state_vector_array_t;
	typedef typename Base::control_vector_array_t control_vector_array_t;
	typedef typename Base::time_array_t time_array_t;

	typedef typename Base::state_matrix_t state_matrix_t;
	typedef typename Base::control_matrix_t control_matrix_t;
	typedef typename Base::state_control_matrix_t state_control_matrix_t;
	typedef typename Base::state_matrix_array_t state_matrix_array_t;
	typedef typename Base::state_control_matrix_array_t state_control_matrix_array_t;


	ShotIntegrator(
			std::shared_ptr<ct::core::ControlledSystem<DerivativeT::STATE_D, DerivativeT::CONTROL_D>> controlledSystem,
			std::shared_ptr<ct::core::LinearSystem<DerivativeT::STATE_D, DerivativeT::CONTROL_D>> linearSystem,
			std::shared_ptr<ct::optcon::CostFunctionQuadratic<DerivativeT::STATE_D, DerivativeT::CONTROL_D>> costFct,
			std::shared_ptr<OptVectorDms<DerivativeT::STATE_D, DerivativeT::CONTROL_D>> w,
			std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
			std::shared_ptr<TimeGrid> timeGrid,
			const size_t shotNr,
			const DmsSettings& settings
			) :
				settings_(settings),
				derivatives_(nullptr),
				integrator_(nullptr)
	{
		derivatives_ = std::allocate_shared<DerivativeT, Eigen::aligned_allocator<DerivativeT>>(
			Eigen::aligned_allocator<DerivativeT>(), controlledSystem, linearSystem, costFct, w, controlSpliner, timeGrid, shotNr, settings);
	}

	virtual ~ShotIntegrator(){}


	virtual void setupSystem() override {
		switch(settings_.integrationType_)
		{
		case DmsSettings::EULER:
		{
			integrator_ = std::allocate_shared<IntegratorEuler, Eigen::aligned_allocator<IntegratorEuler>>
							(Eigen::aligned_allocator<IntegratorEuler>(), derivatives_);
			break;
		}
		case DmsSettings::RK4:
		{
			integrator_ = std::allocate_shared<IntegratorRK4, Eigen::aligned_allocator<IntegratorRK4>>
							(Eigen::aligned_allocator<IntegratorRK4>(), derivatives_);
			break;
		}
		case DmsSettings::RK5:
		{
			integrator_ = std::allocate_shared<IntegratorRK5Var, Eigen::aligned_allocator<IntegratorRK5Var>>
							(Eigen::aligned_allocator<IntegratorRK5Var>(), derivatives_, EventHandlePtrVec(0), settings_.absErrTol_, settings_.relErrTol_);
			break;
		}
		default:
		{
			std::cerr << "... ERROR: unknown integration type. Exiting" << std::endl;
			exit(0);
		}
		}
	}


	virtual void integrate() override {

		// remapping the integration to the individual derivatives classes

		derivatives_->initForIntegration();	// prepare ...

		double t_shot_start = derivatives_->getShotStartTime();
		double t_shot_end = derivatives_->getShotEndTime();

		ct::core::StateVector<DerivativeT::derivativeSize_> initState = derivatives_->getInitState();

		integrator_->integrate_adaptive(initState, t_shot_start, t_shot_end,
				derivatives_->stateTrajectory(),
				derivatives_->timeTrajectory(),
				settings_.dt_sim_);

		derivatives_->wrapUpIntegration(); // ... process variables
	}


	virtual void retrieveStateTrajectories(
			time_array_t& timeTraj,
			state_vector_array_t& stateTraj,
			double& cost) override {

		derivatives_->retrieveStateTrajectories(timeTraj, stateTraj, cost);
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
			double& costGradientHi) override {

		derivatives_->retrieveTrajectories(timeTraj, stateTraj, dXdSiTraj, dXdQiTraj,
				dXdQip1Traj, dXdHiTraj, costGradientSi, costGradientQi, costGradientQip1, costGradientHi);
	}

private:
	DmsSettings settings_;
	std::shared_ptr<DerivativeT> derivatives_;
	std::shared_ptr<IntegratorBase> integrator_;
};

}//namespace optcon
}//namespace ct




#endif /* CT_DMS_SHOTINTEGRATOR_HPP_ */
