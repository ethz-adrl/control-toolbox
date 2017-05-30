/*
 * ShotContainer.hpp
 *
 * Created: 15.01.2016
 * Author: mgiftthaler@ethz.ch
 *
 * */

#ifndef DMS_SHOT_CONTAINER_HPP_
#define DMS_SHOT_CONTAINER_HPP_

#include <cmath>
#include <functional>

#include <ct/core/core.h>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/DmsDimensions.hpp>
#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>
#include <ct/optcon/dms/dms_core/ControllerDms.h>

#include "derivatives/derivatives.hpp"
#include "derivatives/ShotIntegrator.hpp"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class ShotContainer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;

	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	ShotContainer() = delete;

	ShotContainer(
			std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem,
			std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem,
			std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct,
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
			std::shared_ptr<TimeGrid> timeGrid,
			size_t shotNr,
			DmsSettings settings
	):
		controlledSystem_(controlledSystem),
		linearSystem_(linearSystem),
		costFct_(costFct),
		w_(w),
		controlSpliner_(controlSpliner),
		timeGrid_(timeGrid),
		shotNr_(shotNr),
		settings_(settings),
		new_w_counter_integration_(0),
		new_w_counter_integration_with_Sens_(0),
		x_history_(state_vector_array_t(0)),
		t_history_(time_array_t(0)),
		dXdSi_history_(state_matrix_array_t(0)),
		dXdQi_history_(state_control_matrix_array_t(0)),
		dXdQip1_history_(state_control_matrix_array_t(0)),
		dXdHi_history_(state_vector_array_t(0)),
		cost_(0.0),
		costGradientSi_(state_vector_t::Zero()),
		costGradientQi_(control_vector_t::Zero()),
		costGradientQip1_(control_vector_t::Zero()),
		costGradientHi_(0.0)
	{
		if(shotNr_ >= settings.N_) throw std::runtime_error("Dms Shot Integrator: shot index >= settings.N_ - check your settings.");


		if(settings.costEvaluationType_ == DmsSettings::SIMPLE)
			shotIntegrator_ = constructShotIntegrator<DerivativeState<STATE_DIM, CONTROL_DIM>>();
		else if(settings.costEvaluationType_ == DmsSettings::FULL)
			shotIntegrator_ = constructShotIntegrator<DerivativeStateCost<STATE_DIM, CONTROL_DIM>>();
		else
			throw std::runtime_error("Unknown cost evaluation type!");

		if(!settings.integrateSens_)
		{
			if(settings.costEvaluationType_ == DmsSettings::SIMPLE)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeRKn<STATE_DIM, CONTROL_DIM>>();

			else
				throw(std::runtime_error("RKn Derivatives not implemented with Cost-evaluator 'Full'! Switch to integrated sensitivities or simple cost evaluator."));
		}
		else if(settings.integrateSens_)
		{
			if(settings.costEvaluationType_ == DmsSettings::SIMPLE && 
				settings.splineType_ == DmsSettings::ZERO_ORDER_HOLD && 
				settings.objectiveType_ == DmsSettings::KEEP_TIME_AND_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeSimpleZoh<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::SIMPLE && 
				settings.splineType_ == DmsSettings::ZERO_ORDER_HOLD && 
				settings.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeSimpleZohTg<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::SIMPLE && 
				settings.splineType_ == DmsSettings::PIECEWISE_LINEAR && 
				settings.objectiveType_ == DmsSettings::KEEP_TIME_AND_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeSimplePwl<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::SIMPLE && 
				settings.splineType_ == DmsSettings::PIECEWISE_LINEAR && 
				settings.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeSimplePwlTg<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::FULL && 
				settings.splineType_ == DmsSettings::ZERO_ORDER_HOLD && 
				settings.objectiveType_ == DmsSettings::KEEP_TIME_AND_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeFullZoh<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::FULL && 
				settings.splineType_ == DmsSettings::ZERO_ORDER_HOLD && 
				settings.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeFullZohTg<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::FULL && 
				settings.splineType_ == DmsSettings::PIECEWISE_LINEAR && 
				settings.objectiveType_ == DmsSettings::KEEP_TIME_AND_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeFullPwl<STATE_DIM, CONTROL_DIM>>();

			else if(settings.costEvaluationType_ == DmsSettings::FULL && 
				settings.splineType_ == DmsSettings::PIECEWISE_LINEAR && 
				settings.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
				sensitivityIntegrator_ = constructSensitivityIntegrator<DerivativeFullPwlTg<STATE_DIM, CONTROL_DIM>>();
		}

		shotIntegrator_->setupSystem();
		sensitivityIntegrator_->setupSystem();
	}


	// In our setup with quadratic cost functions, we use x_ref = x_final.
	//Therefore, we update x_ref and x_final here
	void updateDesiredState(const state_vector_t& x_final)
	{
		costFct_->updateReferenceState(x_final);
		costFct_->updateFinalState(x_final);
	}

	// integrate shot without calculating state sensitivities
	void integrateShot(double dtInt)
	{
		if(w_->getUpdateCount() != new_w_counter_integration_)
		{
			new_w_counter_integration_ = w_->getUpdateCount();
			shotIntegrator_->integrate(dtInt);
			shotIntegrator_->retrieveStateTrajectories(t_history_, x_history_, cost_);
		}
	}

	// integrate shot forward in time and compute sensitivity on-the-fly
	void integrateShotandComputeSensitivity()
	{
		if(w_->getUpdateCount() != new_w_counter_integration_with_Sens_)
		{
			new_w_counter_integration_with_Sens_ = w_->getUpdateCount();
			sensitivityIntegrator_->integrate(settings_.dt_sim_);
			sensitivityIntegrator_->retrieveTrajectories(t_history_, x_history_, dXdSi_history_, dXdQi_history_, dXdQip1_history_, dXdHi_history_, 
					costGradientSi_, costGradientQi_, costGradientQip1_, costGradientHi_);
		}
	}

	const state_vector_t getStateIntegrated()
	{
		return x_history_.back();
	}

	const double getIntegrationTimeFinal()
	{
		return t_history_.back();
	}

	const state_matrix_t getdXdSiIntegrated()
	{
		return dXdSi_history_.back();
	}

	const state_control_matrix_t getdXdQiIntegrated()
	{
		return dXdQi_history_.back();
	}

	const state_control_matrix_t getdXdQip1Integrated()
	{
		return dXdQip1_history_.back();
	}

	const state_vector_t getdXdHiIntegrated()
	{
		return dXdHi_history_.back();
	}

	const state_vector_array_t& getXHistory() const
	{		
		return x_history_;
	}

	const control_vector_array_t& getUHistory()
	{
		u_history_.clear();
		for(size_t t = 0; t < t_history_.size(); ++t)
		{
			u_history_.push_back(controlSpliner_->evalSpline(t_history_[t], shotNr_));
		}
		return u_history_;
	}

	const time_array_t& getTHistory() const
	{
		return t_history_;
	}

	const double getCostIntegrated() const
	{
		return cost_;
	}

	const state_vector_t getdLdSiIntegrated() const
	{
		return costGradientSi_;
	}

	const control_vector_t getdLdQiIntegrated() const
	{
		return costGradientQi_;
	}

	const control_vector_t getdLdQip1Integrated() const 
	{
		return costGradientQip1_;
	}

	const double getdLdHiIntegrated() const
	{
		return costGradientHi_;
	}

	// return pointer to controlled system (controlled with DMS controller)
	std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> getControlledSystemPtr() {
		return controlledSystem_;
	}


private:
	std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM>> controlledSystem_;
	std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct_;
	std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM>> linearSystem_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
	std::shared_ptr<TimeGrid> timeGrid_;

	const size_t shotNr_; // number/index of this particular shot.
	const DmsSettings settings_;

	size_t new_w_counter_integration_;
	size_t new_w_counter_integration_with_Sens_;

	// Integrated trajectories
	state_vector_array_t x_history_;
	control_vector_array_t u_history_;
	time_array_t t_history_;

	//Sensitivity Trajectories
	state_matrix_array_t dXdSi_history_;
	state_control_matrix_array_t dXdQi_history_;
	state_control_matrix_array_t dXdQip1_history_;
	state_vector_array_t dXdHi_history_;

	//Cost and cost gradient
	double cost_;
	state_vector_t costGradientSi_;
	control_vector_t costGradientQi_;
	control_vector_t costGradientQip1_;
	double costGradientHi_;


	//Integrators
	std::shared_ptr<ShotIntegratorBase<STATE_DIM, CONTROL_DIM>> sensitivityIntegrator_;
	std::shared_ptr<ShotIntegratorBase<STATE_DIM, CONTROL_DIM>> shotIntegrator_;


	// helper function, called from constructor
	template <class C>
	std::shared_ptr<ShotIntegratorBase<STATE_DIM, CONTROL_DIM>> constructShotIntegrator(){
		return std::shared_ptr<ShotIntegrator<C>> (new ShotIntegrator<C> (controlledSystem_, linearSystem_, costFct_, w_, controlSpliner_, timeGrid_, shotNr_ , settings_));
	}

	// helper function, called from constructor
	template <class C>
	std::shared_ptr<ShotIntegratorBase<STATE_DIM, CONTROL_DIM>> constructSensitivityIntegrator(){
		return std::shared_ptr<ShotIntegrator<C>> (new ShotIntegrator<C> (controlledSystem_, linearSystem_, costFct_, w_, controlSpliner_, timeGrid_, shotNr_ , settings_));
	}
};

} // namespace optcon
} // namespace ct

#endif
