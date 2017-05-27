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
		settings_(settings),
		tf_(0.0),
		x0_(x0)
	{
		assert(systemPtrs.size() == settings_.N_);
		assert(linearPtrs.size() == settings_.N_);
		assert(costPtrs.size() == settings_.N_);
		settings_.parametersOk();

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
				new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM> (constraintsIntermediate.front(), activeInd, settings_.N_ + 1));
		}

		if(constraintsFinal.size() > 0)
		{
			std::vector<size_t> activeInd;
			activeInd.push_back(settings_.N_);
			std::cout << "setting up final constraints" << std::endl;
			constraintsFinalLocal_ = std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> (
				new ConstraintDiscretizer<STATE_DIM, CONTROL_DIM> (constraintsFinal.front(), activeInd, settings_.N_ + 1));
		}


		for (size_t shotIdx = 0; shotIdx < settings_.N_; shotIdx++)
		{
			std::shared_ptr<ControllerDms<STATE_DIM, CONTROL_DIM>> newController ( new ControllerDms<STATE_DIM, CONTROL_DIM>(optVariablesDms_, shotIdx));
			systemPtrs[shotIdx]->setController(newController);
			linearPtrs[shotIdx]->setController(newController);
			shotContainers_.push_back(std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>
				( new ShotContainer<STATE_DIM, CONTROL_DIM> (shotIdx, systemPtrs[shotIdx], optVariablesDms_, settings_, costPtrs[shotIdx], linearPtrs[shotIdx])));
		}

		switch (settings_.costEvaluationType_)
		{
			case DmsSettings::SIMPLE:
			{
				costEvaluator_ = std::shared_ptr<CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>> (
						new CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>(costPtrs.front(), optVariablesDms_, settings_)); break;
			}
			case DmsSettings::FULL:
			{
				costEvaluator_ = std::shared_ptr<CostEvaluatorFull<STATE_DIM, CONTROL_DIM>>(
						new CostEvaluatorFull<STATE_DIM, CONTROL_DIM> (costPtrs.front(), shotContainers_, optVariablesDms_, settings_));	break;
			}
			default:
				throw(std::runtime_error("ERROR: Unknown cost evaluation type in dms.hpp! "));
		}

		optVariables_ = optVariablesDms_;

		constraints_ = std::shared_ptr<ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>> (
			new ConstraintsContainerDms<STATE_DIM, CONTROL_DIM>(shotContainers_, optVariablesDms_, x0, settings_, constraintsIntermediateLocal_, constraintsFinalLocal_));

		optVariables_->resizeConstraintVars(getConstraintsCount());
	}

	virtual ~DmsProblem(){}

	virtual void updateProblem() override
	{
		optVariablesDms_->update();

		if(constraintsIntermediateLocal_)
			constraintsIntermediateLocal_->updateTrajectories(
				optVariablesDms_->getStateSolution(), optVariablesDms_->getInputSolution(), optVariablesDms_->getTimeSolution());

		if(constraintsFinalLocal_)
			constraintsFinalLocal_->updateTrajectories(
				optVariablesDms_->getStateSolution(), optVariablesDms_->getInputSolution(), optVariablesDms_->getTimeSolution());
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
		return optVariablesDms_->getStateSolution();
	}

	const control_vector_array_t& getInputSolution()
	{
		return optVariablesDms_->getInputSolution();
	}

	const time_array_t& getTimeSolution()
	{
		return optVariablesDms_->getTimeSolution();
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
		optVariablesDms_->setInitGuess(x_init_guess, u_init_guess, t_init_guess);

		if(constraintsIntermediateLocal_)
			constraintsIntermediateLocal_->updateTrajectories(
				optVariablesDms_->getStateSolution(), optVariablesDms_->getInputSolution(), optVariablesDms_->getTimeSolution());

		if(constraintsFinalLocal_)
			constraintsFinalLocal_->updateTrajectories(
				optVariablesDms_->getStateSolution(), optVariablesDms_->getInputSolution(), optVariablesDms_->getTimeSolution());		
	}

	const core::Time getTimeHorizon() const 
	{
		return tf_;
	}

	void changeTimeHorizon(const core::Time tf)
	{
		tf_ = tf;
	}

	void changeInitialState(const state_vector_t& x0)
	{
		x0_ = x0;
	}

	void printSolution()
	{
		optVariablesDms_->printoutSolution();
	}


private:
	DmsSettings settings_;
	
	core::Time tf_;
	state_vector_t x0_;

	std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsIntermediateLocal_;
	std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM>> constraintsFinalLocal_;
	std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>>> shotContainers_;
	std::shared_ptr<OptConProblem_t> optconProblem_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> optVariablesDms_;

	state_vector_array_t stateSolutionDense_;
	control_vector_array_t inputSolutionDense_;
	time_array_t timeSolutionDense_;
};


} // namespace optcon
} // namespace ct


#endif // CT_OPTCON_DMS_PROBLEM_HPP_