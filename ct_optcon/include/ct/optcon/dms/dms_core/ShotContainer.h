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

#ifndef CT_OPTCON_DMS_CORE_SHOT_CONTAINER_H_
#define CT_OPTCON_DMS_CORE_SHOT_CONTAINER_H_

#include <cmath>
#include <functional>

#include <ct/core/core.h>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ControllerDms.h>

#include "derivatives/derivatives.h"
#include "derivatives/ShotIntegrator.h"

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      This class performs the state and the sensitivity integration on
 *             a shot
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
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

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  controlledSystem  The nonlinear system
	 * @param[in]  linearSystem      The linearized system
	 * @param[in]  costFct           The costfunction
	 * @param[in]  w                 The optimization vector
	 * @param[in]  controlSpliner    The control input spliner
	 * @param[in]  timeGrid          The timegrid 
	 * @param[in]  shotNr            The shot number
	 * @param[in]  settings          The dms settings
	 */
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

	/**
	 * @brief      Performs the state integration between the shots
	 */
	void integrateShot()
	{
		if(w_->getUpdateCount() != new_w_counter_integration_)
		{
			new_w_counter_integration_ = w_->getUpdateCount();
			shotIntegrator_->integrate();
			shotIntegrator_->retrieveStateTrajectories(t_history_, x_history_, cost_);
		}
	}

	/**
	 * @brief      Performs the state and the sensitivity integration between the shots
	 */
	void integrateShotandComputeSensitivity()
	{
		if(w_->getUpdateCount() != new_w_counter_integration_with_Sens_)
		{
			new_w_counter_integration_with_Sens_ = w_->getUpdateCount();
			sensitivityIntegrator_->integrate();
			sensitivityIntegrator_->retrieveTrajectories(t_history_, x_history_, dXdSi_history_, dXdQi_history_, dXdQip1_history_, dXdHi_history_, 
					costGradientSi_, costGradientQi_, costGradientQip1_, costGradientHi_);
		}
	}

	/**
	 * @brief      Returns the integrated state
	 *
	 * @return     The integrated state
	 */
	const state_vector_t getStateIntegrated()
	{
		return x_history_.back();
	}

	/**
	 * @brief      Returns the end time of the integration	
	 *
	 * @return     The end time of the integration.
	 */
	const double getIntegrationTimeFinal()
	{
		return t_history_.back();
	}

	/**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             discretized state s_i
	 *
	 * @return     The integrated sensitivity
	 */
	const state_matrix_t getdXdSiIntegrated()
	{
		return dXdSi_history_.back();
	}

	/**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             discretized inputs q_i
	 *
	 * @return     The integrated sensitivity
	 */
	const state_control_matrix_t getdXdQiIntegrated()
	{
		return dXdQi_history_.back();
	}

	/**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             discretized inputs q_{i+1}
	 *
	 * @return     The integrated sensitivity
	 */
	const state_control_matrix_t getdXdQip1Integrated()
	{
		return dXdQip1_history_.back();
	}

	/**
	 * @brief      Returns the integrated ODE sensitivity with respect to the
	 *             time segments h_i
	 *
	 * @return     The integrated sensitivity
	 */
	const state_vector_t getdXdHiIntegrated()
	{
		return dXdHi_history_.back();
	}

	/**
	 * @brief      Gets the full integrated state trajectory.
	 *
	 * @return     The integrated state trajectory
	 */
	const state_vector_array_t& getXHistory() const
	{		
		return x_history_;
	}

	/**
	 * @brief      Returns the control input trajectory used during the state integration
	 *
	 * @return     The control trajectory
	 */
	const control_vector_array_t& getUHistory()
	{
		u_history_.clear();
		for(size_t t = 0; t < t_history_.size(); ++t)
		{
			u_history_.push_back(controlSpliner_->evalSpline(t_history_[t], shotNr_));
		}
		return u_history_;
	}

	/**
	 * @brief      Returns the time trajectory used during the integration
	 *
	 * @return     The time trajectory
	 */
	const time_array_t& getTHistory() const
	{
		return t_history_;
	}

	/**
	 * @brief      Gets the cost integrated.
	 *
	 * @return     The integrated cost.
	 */
	const double getCostIntegrated() const
	{
		return cost_;
	}

	/**
	 * @brief      Returns the cost gradient with respect to s_i integrated over
	 *             the shot
	 *
	 * @return     The cost gradient
	 */
	const state_vector_t getdLdSiIntegrated() const
	{
		return costGradientSi_;
	}

	/**
	 * @brief      Returns the cost gradient with respect to q_i integrated over
	 *             the shot
	 *
	 * @return     The cost gradient
	 */
	const control_vector_t getdLdQiIntegrated() const
	{
		return costGradientQi_;
	}

	/**
	 * @brief      Returns to cost gradient with respect to q_{i+1} integrated
	 *             over the shot
	 *
	 * @return     The cost gradient
	 */
	const control_vector_t getdLdQip1Integrated() const 
	{
		return costGradientQip1_;
	}

	/**
	 * @brief      Returns to cost gradient with respect to h_i integrated over
	 *             the shot
	 *
	 * @return     The cost gradient
	 */
	const double getdLdHiIntegrated() const
	{
		return costGradientHi_;
	}

	/**
	 * @brief      Returns a pointer to the nonlinear dynamics used for this
	 *             shot
	 *
	 * @return     The pointer to the nonlinear dynamics
	 */
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

	const size_t shotNr_; 
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


	/**
	 * @brief      Returns a new shot integrator depending on the derivative type C
	 *
	 * @return     The new shot integrator
	 *
	 * @tparam     C    The derivative type
	 */
	template <class C>
	std::shared_ptr<ShotIntegratorBase<STATE_DIM, CONTROL_DIM>> constructShotIntegrator(){
		return std::shared_ptr<ShotIntegrator<C>> (new ShotIntegrator<C> (controlledSystem_, linearSystem_, costFct_, w_, controlSpliner_, timeGrid_, shotNr_ , settings_));
	}

	/**
	 * @brief      Returns a new integrator integrating the sensitivities
	 *
	 * @return     The new sensitivity integrator
	 *
	 * @tparam     C    The derivative type
	 */
	template <class C>
	std::shared_ptr<ShotIntegratorBase<STATE_DIM, CONTROL_DIM>> constructSensitivityIntegrator(){
		return std::shared_ptr<ShotIntegrator<C>> (new ShotIntegrator<C> (controlledSystem_, linearSystem_, costFct_, w_, controlSpliner_, timeGrid_, shotNr_ , settings_));
	}
};

} // namespace optcon
} // namespace ct

#endif //CT_OPTCON_DMS_CORE_SHOT_CONTAINER_H_
