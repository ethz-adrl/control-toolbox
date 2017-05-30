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


#ifndef CT_OPTCON_DMS_PROBLEM_HPP_
#define CT_OPTCON_DMS_PROBLEM_HPP_


#include <ct/optcon/problem/OptConProblem.h>
#include <ct/optcon/dms/dms_core/DmsDimensions.hpp>
#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>
#include <ct/optcon/dms/dms_core/ControllerDms.h>
#include <ct/optcon/dms/constraints/ConstraintsContainerDms.hpp>
#include <ct/optcon/dms/constraints/ConstraintDiscretizer.hpp>

#include <ct/optcon/dms/dms_core/cost_evaluator/CostEvaluatorSimple.hpp>
#include <ct/optcon/dms/dms_core/cost_evaluator/CostEvaluatorFull.hpp>
#include <ct/optcon/dms/dms_core/DmsSettings.hpp>

#include <ct/optcon/nlp/Nlp.h>

namespace ct {
namespace optcon {



/**
 * @brief      Testing stuff.
 *
 * @tparam     STATE_DIM    { what a crazy parameter }
 * @tparam     CONTROL_DIM  { this one is even crazier }
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class DmsProblem : public Nlp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
	typedef Eigen::Map<VectorXd> MapVecXd;
	typedef Eigen::Map<const VectorXd> MapConstVecXd;

	typedef OptConProblem<STATE_DIM, CONTROL_DIM> OptConProblem_t;



	

	/**
	 * @brief      { Sets up the Direct Multiple Shooting Problem }
	 *
	 * @param[in]  settings                 The settings
	 * @param[in]  systemPtrs               The system ptrs
	 * @param[in]  linearPtrs               The linear ptrs
	 * @param[in]  costPtrs                 The cost ptrs
	 * @param[in]  constraintsIntermediate  The constraints intermediate
	 * @param[in]  constraintsFinal         The constraints final
	 * @param[in]  x0                       The x 0
	 */
	DmsProblem(
			DmsSettings settings,
			std::vector<typename OptConProblem_t::DynamicsPtr_t> systemPtrs,
			std::vector<typename OptConProblem_t::LinearPtr_t> linearPtrs,
			std::vector<typename OptConProblem_t::CostFunctionPtr_t> costPtrs,
			std::vector<typename OptConProblem_t::ConstraintPtr_t> constraintsIntermediate,
			std::vector<typename OptConProblem_t::ConstraintPtr_t> constraintsFinal,
			const state_vector_t& x0
		) 
	:
		settings_(settings)
	{
		assert(systemPtrs.size() == settings_.N_);
		assert(linearPtrs.size() == settings_.N_);
		assert(costPtrs.size() == settings_.N_);
		settings_.parametersOk();

		timeGrid_ = std::shared_ptr<TimeGrid> (new TimeGrid (settings.N_, settings.T_));

		// initialize the spliner for control inputs
		switch(settings_.splineType_)
		{
			case DmsSettings::ZERO_ORDER_HOLD:
			{
				controlSpliner_ = std::shared_ptr<ZeroOrderHoldSpliner<control_vector_t>>(
						new ZeroOrderHoldSpliner<control_vector_t> (timeGrid_));
				break;
			}
			case DmsSettings::PIECEWISE_LINEAR:
			{
				controlSpliner_ = std::shared_ptr<LinearSpliner<control_vector_t>>(
						new LinearSpliner<control_vector_t> (timeGrid_));
				break;
			}
			default:
				throw(std::runtime_error("specified invalid spliner type in OptVectorDms-class"));
		}


		size_t wLength = (settings.N_ + 1)*(STATE_DIM + CONTROL_DIM);
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
			wLength += settings_.N_;

		optVariablesDms_ = std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>>(
				new OptVectorDms<STATE_DIM, CONTROL_DIM>(wLength, settings));

		// We can decide at how many shots the constraints will be active
		if(constraintsIntermediate.size() > 0)
		{
			std::cout << "Setting up intermediate constraints" << std::endl;
			std::vector<size_t> activeInd;
			for(size_t i = 0; i < settings_.N_ + 1; ++i)
				activeInd.push_back(i);
			constraintsIntermediateLocal_ = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> (
				new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM> (optVariablesDms_, constraintsIntermediate.front(), activeInd, settings_.N_ + 1));
		}

		if(constraintsFinal.size() > 0)
		{
			std::vector<size_t> activeInd;
			activeInd.push_back(settings_.N_);
			std::cout << "setting up final constraints" << std::endl;
			constraintsFinalLocal_ = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> (
				new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM> (optVariablesDms_, constraintsFinal.front(), activeInd, settings_.N_ + 1));
		}


		for (size_t shotIdx = 0; shotIdx < settings_.N_; shotIdx++)
		{
			std::shared_ptr<ControllerDms<STATE_DIM, CONTROL_DIM>> newController ( new ControllerDms<STATE_DIM, CONTROL_DIM>(controlSpliner_, shotIdx));
			systemPtrs[shotIdx]->setController(newController);
			linearPtrs[shotIdx]->setController(newController);
			shotContainers_.push_back(std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>
				( new ShotContainer<STATE_DIM, CONTROL_DIM> (
					systemPtrs[shotIdx], linearPtrs[shotIdx], costPtrs[shotIdx], optVariablesDms_, controlSpliner_, timeGrid_,  shotIdx, settings_)));
		}

		switch (settings_.costEvaluationType_)
		{
			case DmsSettings::SIMPLE:
			{
				costEvaluator_ = std::shared_ptr<CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>> (
						new CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>(costPtrs.front(), optVariablesDms_, timeGrid_, settings_)); break;
			}
			case DmsSettings::FULL:
			{
				costEvaluator_ = std::shared_ptr<CostEvaluatorFull<STATE_DIM, CONTROL_DIM>>(
						new CostEvaluatorFull<STATE_DIM, CONTROL_DIM> (costPtrs.front(),optVariablesDms_ , controlSpliner_, shotContainers_, settings_));	break;
			}
			default:
				throw(std::runtime_error("ERROR: Unknown cost evaluation type in dms.hpp! "));
		}

		optVariables_ = optVariablesDms_;

		constraints_ = std::shared_ptr<ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>> (
			new ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>(optVariablesDms_, timeGrid_, shotContainers_, constraintsIntermediateLocal_, constraintsFinalLocal_, x0, settings_));

		optVariables_->resizeConstraintVars(getConstraintsCount());
	}

	virtual ~DmsProblem(){}

	virtual void updateProblem() override
	{
		controlSpliner_->computeSpline(optVariablesDms_->getOptimizedInputs().toImplementation());
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
			timeGrid_->updateTimeGrid(optVariablesDms_->getOptimizedTimeSegments());

		if(constraintsIntermediateLocal_)
			constraintsIntermediateLocal_->updateTrajectories(
				optVariablesDms_->getOptimizedStates(), optVariablesDms_->getOptimizedInputs(), timeGrid_->toImplementation());

		if(constraintsFinalLocal_)
			constraintsFinalLocal_->updateTrajectories(
				optVariablesDms_->getOptimizedStates(), optVariablesDms_->getOptimizedInputs(), timeGrid_->toImplementation());
	}

	// end of methods needed in nlp solver
	// Methods needed in Dms Solver

	void configure(const DmsSettings& settings)
	{
		// Perform other initialization stuff here
		settings_ = settings;
	}


	const state_vector_array_t& getStateSolution()
	{
		return optVariablesDms_->getOptimizedStates();
	}

	const control_vector_array_t& getInputSolution()
	{
		return optVariablesDms_->getOptimizedInputs();
	}

	const time_array_t& getTimeSolution()
	{
		return timeGrid_->toImplementation();
	}

	const state_vector_array_t& getStateTrajectory()
	{
		stateSolutionDense_.clear();
		for(auto shotContainer : shotContainers_)
		{
			state_vector_array_t x_traj = shotContainer->getXHistory();
			for(size_t j = 0; j < x_traj.size(); ++j)
				stateSolutionDense_.push_back(x_traj[j]);
		}
		return stateSolutionDense_;
	}

	const control_vector_array_t& getInputTrajectory()
	{
		inputSolutionDense_.clear();
		for(auto shotContainer : shotContainers_)
		{
			control_vector_array_t u_traj = shotContainer->getUHistory();
			for(size_t j = 0; j < u_traj.size(); ++j)
				inputSolutionDense_.push_back(u_traj[j]);
		}
		return inputSolutionDense_;
	}

	const time_array_t& getTimeArray()
	{
		timeSolutionDense_.clear();
		for(auto shotContainer : shotContainers_)
		{
			time_array_t t_traj = shotContainer->getTHistory();
			for(size_t j = 0; j < t_traj.size(); ++j)
				timeSolutionDense_.push_back(t_traj[j]);
		}
		return timeSolutionDense_;
	}

	void setInitialGuess(
		const state_vector_array_t& x_init_guess,
		const control_vector_array_t& u_init_guess,
		const time_array_t& t_init_guess = time_array_t(0.0))
	{
		optVariablesDms_->setInitGuess(x_init_guess, u_init_guess);

		if(constraintsIntermediateLocal_)
			constraintsIntermediateLocal_->updateTrajectories(
				optVariablesDms_->getOptimizedStates(), optVariablesDms_->getOptimizedInputs(), timeGrid_->toImplementation());

		if(constraintsFinalLocal_)
			constraintsFinalLocal_->updateTrajectories(
				optVariablesDms_->getOptimizedStates(), optVariablesDms_->getOptimizedInputs(), timeGrid_->toImplementation());		
	}

	const core::Time getTimeHorizon() const 
	{
		return timeGrid_->getTimeHorizon();
	}

	void changeTimeHorizon(const core::Time tf)
	{
		timeGrid_->changeTimeHorizon(tf);
	}

	void changeInitialState(const state_vector_t& x0)
	{
		optVariablesDms_->changeInitialState(x0);
	}

	void printSolution()
	{
		optVariablesDms_->printoutSolution();
	}


private:
	DmsSettings settings_;
	
	std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsIntermediateLocal_;
	std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsFinalLocal_;
	std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers_;
	std::shared_ptr<OptConProblem_t> optconProblem_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> optVariablesDms_;

	/* splining */
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
	std::shared_ptr<TimeGrid> timeGrid_;


	state_vector_array_t stateSolutionDense_;
	control_vector_array_t inputSolutionDense_;
	time_array_t timeSolutionDense_;
};


} // namespace optcon
} // namespace ct


#endif // CT_OPTCON_DMS_PROBLEM_HPP_