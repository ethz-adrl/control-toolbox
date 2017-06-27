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


#ifndef CT_OPTCON_DMS_DMS_CORE_PROBLEM_H_
#define CT_OPTCON_DMS_DMS_CORE_PROBLEM_H_


#include <ct/optcon/problem/OptConProblem.h>
#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ControllerDms.h>
#include <ct/optcon/dms/constraints/ConstraintsContainerDms.h>
#include <ct/optcon/dms/constraints/ConstraintDiscretizer.h>

#include <ct/optcon/dms/dms_core/cost_evaluator/CostEvaluatorSimple.h>
#include <ct/optcon/dms/dms_core/cost_evaluator/CostEvaluatorFull.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>

#include <ct/optcon/nlp/Nlp.h>

namespace ct {
namespace optcon {



/**
 * @ingroup    DMS
 *
 * @brief      This class sets up the DMS problem
 *
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
	 * @brief      Custom constructor, sets up the objects needed for the Dms
	 *             algorithm depending on the settings
	 *
	 * @param[in]  settings                 The Dms settings
	 * @param[in]  systemPtrs               The non linear systems
	 * @param[in]  linearPtrs               The linearized systems
	 * @param[in]  costPtrs                 The cost function
	 * @param[in]  stateInputConstraints  The intermediate constraints
	 * @param[in]  pureStateConstraints         The final constraints
	 * @param[in]  x0                       The initial state
	 */
	DmsProblem(
			DmsSettings settings,
			std::vector<typename OptConProblem_t::DynamicsPtr_t> systemPtrs,
			std::vector<typename OptConProblem_t::LinearPtr_t> linearPtrs,
			std::vector<typename OptConProblem_t::CostFunctionPtr_t> costPtrs,
			std::vector<typename OptConProblem_t::ConstraintPtr_t> stateInputConstraints,
			std::vector<typename OptConProblem_t::ConstraintPtr_t> pureStateConstraints,
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
				throw(std::runtime_error("Unknown spline type"));
		}


		size_t wLength = (settings.N_ + 1)*(STATE_DIM + CONTROL_DIM);
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
			wLength += settings_.N_;

		optVariablesDms_ = std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>>(
				new OptVectorDms<STATE_DIM, CONTROL_DIM>(wLength, settings));
			
		if(stateInputConstraints.size() > 0 || pureStateConstraints.size() > 0)
			discretizedConstraints_ = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> (
				new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM> (optVariablesDms_, controlSpliner_, timeGrid_, settings_.N_));

		if(stateInputConstraints.size() > 0)
			discretizedConstraints_->setStateInputConstraints(stateInputConstraints.front());

		if(pureStateConstraints.size() > 0)
			discretizedConstraints_->setPureStateConstraints(pureStateConstraints.front());

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
				throw(std::runtime_error("Unknown cost evaluation type"));
		}

		optVariables_ = optVariablesDms_;

		constraints_ = std::shared_ptr<ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>> (
			new ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>(optVariablesDms_, timeGrid_, shotContainers_, discretizedConstraints_, x0, settings_));

		optVariables_->resizeConstraintVars(getConstraintsCount());
	}

	/**
	 * @brief      Destructor
	 */
	virtual ~DmsProblem(){}

	virtual void updateProblem() override
	{
		controlSpliner_->computeSpline(optVariablesDms_->getOptimizedInputs().toImplementation());
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
		{
			// std::cout << "optVariablesDms_->getOptimizedTimeSegments(): " << optVariablesDms_->getOptimizedTimeSegments().transpose() << std::endl;
			timeGrid_->updateTimeGrid(optVariablesDms_->getOptimizedTimeSegments());
		}
	}

	/**
	 * @brief      Updates the settings
	 *
	 * @param[in]  settings  New dms settings
	 */
	void configure(const DmsSettings& settings)
	{
		settings_ = settings;
	}

	/**
	 * @brief      Retrieves the solution state trajectory at every shot
	 *
	 * @return     The state solution trajectory
	 */
	const state_vector_array_t& getStateSolution()
	{
		return optVariablesDms_->getOptimizedStates();
	}

	/**
	 * @brief      Retrieves the solution control trajectory at every shot
	 *
	 * @return     The control solution trajectory
	 */
	const control_vector_array_t& getInputSolution()
	{
		return optVariablesDms_->getOptimizedInputs();
	}

	/**
	 * @brief      Retrieves the solution time trajectory at every shot
	 *
	 * @return     The time solution trajectory
	 */
	const time_array_t& getTimeSolution()
	{
		return timeGrid_->toImplementation();
	}

	/**
	 * @brief      Retrieves a dense state solution trajectory
	 *
	 * @return     The dense state solution trajectory
	 */
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

	/**
	 * @brief      Retrieves a dense input solution trajectory
	 *
	 * @return     The dense control solution trajectory
	 */
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

	/**
	 * @brief      Retrieves a dense time solution trajectory
	 *
	 * @return     The dense time solution trajectory
	 */
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

	/**
	 * @brief      Sets the initial guess of the optimization
	 *
	 * @param[in]  x_init_guess  The state trajectory initial guess
	 * @param[in]  u_init_guess  The control trajectory initial guess
	 * @param[in]  t_init_guess  The time trajectory initial guess
	 */
	void setInitialGuess(
		const state_vector_array_t& x_init_guess,
		const control_vector_array_t& u_init_guess,
		const time_array_t& t_init_guess = time_array_t(0.0))
	{
		optVariablesDms_->setInitGuess(x_init_guess, u_init_guess);	
	}

	/**
	 * @brief      Return the timehorizon of the problem
	 *
	 * @return     The time horizon
	 */
	const core::Time getTimeHorizon() const 
	{
		return timeGrid_->getTimeHorizon();
	}

	/**
	 * @brief      Updates the timehorizon
	 *
	 * @param[in]  tf    The new time horizon
	 */
	void changeTimeHorizon(const core::Time tf)
	{
		timeGrid_->changeTimeHorizon(tf);
	}

	/**
	 * @brief      Updates the initial state
	 *
	 * @param[in]  x0    The new inital state
	 */
	void changeInitialState(const state_vector_t& x0)
	{
		// constraints_->changeInitialConstraint(x0);
		optVariablesDms_->changeInitialState(x0);
	}

	/**
	 * @brief      Prints the solution trajectories
	 */
	void printSolution()
	{
		optVariablesDms_->printoutSolution();
	}


private:
	DmsSettings settings_;
	
	std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> discretizedConstraints_;

	std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers_; 
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> optVariablesDms_; 
	std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_; 
	std::shared_ptr<TimeGrid> timeGrid_; 

	state_vector_array_t stateSolutionDense_; 
	control_vector_array_t inputSolutionDense_;
	time_array_t timeSolutionDense_;
};


} // namespace optcon
} // namespace ct


#endif // CT_OPTCON_DMS_PROBLEM_H_