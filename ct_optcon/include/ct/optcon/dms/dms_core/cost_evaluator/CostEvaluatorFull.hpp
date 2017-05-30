#ifndef DMS_COST_EVALUATOR_FULL_HPP_
#define DMS_COST_EVALUATOR_FULL_HPP_

#include <omp.h>
#include <math.h>
#include <cmath>
#include <functional>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>
#include <ct/optcon/dms/dms_core/ShotContainer.hpp>
#include <ct/optcon/nlp/DiscreteCostEvaluatorBase.h>

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>
#include <ct/core/types/trajectories/StateVectorArray.h>
#include <ct/core/types/trajectories/ControlVectorArray.h>
#include <ct/core/types/Time.h>

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
class CostEvaluatorFull : public DiscreteCostEvaluatorBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;	
	typedef typename DIMENSIONS::time_array_t time_array_t;	

	CostEvaluatorFull() = delete;

	CostEvaluatorFull(
			std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct,
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
			std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotInt,
			DmsSettings settings):
				costFct_(costFct),
				w_(w),
				controlSpliner_(controlSpliner),
				shotContainers_(shotInt),
				settings_(settings)
	{}

	virtual ~CostEvaluatorFull(){}

	virtual double eval() override
	{
		double cost = 0.0;
		// go through all shots, integrate the state trajectories and evaluate cost accordingly
		// intermediate costs
		#pragma omp parallel for num_threads( settings_.nThreads_ )
		for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
			(*shotContainer)->integrateShot(settings_.dt_sim_);	
		}

		for(auto shotContainer : shotContainers_)
			cost += shotContainer->getCostIntegrated();	
			
		// terminal cost
		costFct_->setCurrentStateAndControl(w_->getOptimizedState(shotContainers_.size()), control_vector_t::Zero());
		cost += costFct_->evaluateTerminal();
		return cost;
	}

	void evalGradient(size_t grad_length, Eigen::Map<Eigen::VectorXd>& grad)
	{
		grad.setZero();

		assert(shotContainers_.size() == settings_.N_);

		// go through all shots, integrate the state trajectories and evaluate cost accordingly
		// intermediate costs
		#pragma omp parallel for num_threads( settings_.nThreads_ )
		for(auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer){
			(*shotContainer)->integrateShotandComputeSensitivity();
		}

		for (size_t shotNr = 0; shotNr< shotContainers_.size(); ++shotNr)
		{
			switch (settings_.splineType_)
			{
				case DmsSettings::ZERO_ORDER_HOLD:
				{
						grad.segment(w_->getStateIndex(shotNr), STATE_DIM) += shotContainers_[shotNr]->getdLdSiIntegrated();
						grad.segment(w_->getControlIndex(shotNr), CONTROL_DIM) += shotContainers_[shotNr]->getdLdQiIntegrated();
					break;
				}
				case DmsSettings::PIECEWISE_LINEAR:
				{
						grad.segment(w_->getStateIndex(shotNr), STATE_DIM) += shotContainers_[shotNr]->getdLdSiIntegrated();
						grad.segment(w_->getControlIndex(shotNr), CONTROL_DIM) += shotContainers_[shotNr]->getdLdQiIntegrated();
						grad.segment(w_->getControlIndex(shotNr+1), CONTROL_DIM) += shotContainers_[shotNr]->getdLdQip1Integrated();
					break;
				}
				default:
					throw(std::runtime_error(" cost gradient not yet implemented for this type of interpolation. Exiting"));
			}

			// H-part.
			if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
			{
				costFct_->setCurrentStateAndControl(shotContainers_[shotNr]->getStateIntegrated(),
													controlSpliner_->evalSpline(shotContainers_[shotNr]->getIntegrationTimeFinal(), shotNr));
				grad(w_->getTimeSegmentIndex(shotNr)) = costFct_->evaluateIntermediate() + shotContainers_[shotNr]->getdLdHiIntegrated();
			}
		}

		/* gradient of terminal cost */
		costFct_->setCurrentStateAndControl(w_->getOptimizedState(shotContainers_.size()), control_vector_t::Zero());
		grad.segment(w_->getStateIndex(settings_.N_), STATE_DIM) += costFct_->stateDerivativeTerminal();// * dXdSi.back();
	}

private:

	std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
	std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers_;

	const DmsSettings settings_;
};

} // namespace optcon
} // namespace ct

#endif
