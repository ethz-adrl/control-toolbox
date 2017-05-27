#ifndef CT_OPTCON_DMS_SOLVER_HPP_
#define CT_OPTCON_DMS_SOLVER_HPP_

#include <ct/optcon/problem/OptConProblem.h>
#include <ct/optcon/solver/OptConSolver.h>

#include <ct/optcon/dms/dms_core/DmsProblem.h>
#include <ct/optcon/dms/dms_core/DmsSettings.hpp>

#include <ct/optcon/nlp/Nlp.h>
#include <ct/optcon/nlp/solver/NlpSolver.h>
#include <ct/optcon/nlp/solver/IpoptSolver.h>
#include <ct/optcon/nlp/solver/SnoptSolver.h>
#include <ct/optcon/nlp/solver/NlpSolverSettings.h>

#include <memory>

namespace ct {
namespace optcon {


template<size_t STATE_DIM, size_t CONTROL_DIM>
struct DmsPolicy
{
	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;	
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	state_vector_array_t xSolution_;
	control_vector_array_t uSolution_;
	time_array_t tSolution_;
};

template <size_t STATE_DIM, size_t CONTROL_DIM>
class DmsSolver : public OptConSolver<DmsSolver<STATE_DIM, CONTROL_DIM>,  DmsPolicy<STATE_DIM, CONTROL_DIM>, DmsSettings, STATE_DIM, CONTROL_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef OptConSolver<DmsSolver<STATE_DIM, CONTROL_DIM>, DmsPolicy<STATE_DIM, CONTROL_DIM>, DmsSettings, STATE_DIM, CONTROL_DIM> Base;

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef DmsPolicy<STATE_DIM, CONTROL_DIM> Policy_t;

	DmsSolver(const OptConProblem<STATE_DIM, CONTROL_DIM> problem, DmsSettings settingsDms) 
	:
	nlpSolver_(nullptr),
	settings_(settingsDms)
	{
		// Create system, linearsystem and costfunction instances 
		this->setProblem(problem);
		dmsProblem_ = std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM>> (new DmsProblem<STATE_DIM, CONTROL_DIM>
				(settingsDms, this->systems_, this->linearSystems_, this->costFunctions_, 
					this->constraintsIntermediate_, this->constraintsFinal_, x0_));

		if(settingsDms.nlpSettings_.solverType_ == NlpSolverSettings::SNOPT)
			nlpSolver_ = std::shared_ptr<SnoptSolver>(new SnoptSolver(dmsProblem_, settingsDms.nlpSettings_));
		else if (settingsDms.nlpSettings_.solverType_ == NlpSolverSettings::IPOPT)
			nlpSolver_ = std::shared_ptr<IpoptSolver> (new IpoptSolver(dmsProblem_, settingsDms.nlpSettings_));
		else
			std::cout << "Unknown solver type... Exiting" << std::endl;

		configure(settingsDms);
	}

	virtual ~DmsSolver(){}

	// initializes the settings dependent stuff...
	virtual void configure(const DmsSettings& settings) override
	{
		dmsProblem_->configure(settings); // only sets settings
		dmsProblem_->changeTimeHorizon(tf_);
		dmsProblem_->changeInitialState(x0_);
		nlpSolver_->configure(settings_.nlpSettings_); // initializes the solver
	}

	virtual bool solve() override
	{
		return nlpSolver_->solve();
	}

	virtual const Policy_t& getSolution() override
	{
		policy_.xSolution_ = dmsProblem_->getStateSolution();
		policy_.uSolution_ = dmsProblem_->getInputSolution();
		policy_.tSolution_ = dmsProblem_->getTimeSolution();
		return policy_;
	}

	virtual const core::StateTrajectory<STATE_DIM> getStateTrajectory() const override {
		return core::StateTrajectory<STATE_DIM>(dmsProblem_->getTimeArray(), dmsProblem_->getStateTrajectory());
	}

	virtual const core::ControlTrajectory<CONTROL_DIM> getControlTrajectory() const override {
		return core::ControlTrajectory<CONTROL_DIM>(dmsProblem_->getTimeArray(), dmsProblem_->getInputTrajectory());
	}

	virtual const core::TimeArray& getTimeArray() const override {
		return dmsProblem_->getTimeArray();
	}

	virtual void setInitialGuess(const Policy_t& initialGuess) override
	{
		dmsProblem_->setInitialGuess(initialGuess.xSolution_, initialGuess.uSolution_);
	}

	virtual core::Time getTimeHorizon() const override
	{
		return dmsProblem_->getTimeHorizon();
	}


	virtual void changeTimeHorizon(const core::Time& tf) override
	{
		tf_ = tf;
	}

	virtual void changeInitialState(const core::StateVector<STATE_DIM>& x0) override
	{
		x0_ = x0;
	}

	virtual void changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf) override
	{
		this->getCostFunctionInstances().resize(settings_.N_);
		if(cf)
			for (size_t i = 0; i<settings_.N_; i++)
				this->getCostFunctionInstances()[i] = typename Base::OptConProblem_t::CostFunctionPtr_t(cf->clone());
	}

	virtual void changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn) override
	{
		this->getNonlinearSystemsInstances().resize(settings_.N_);

		if(dyn)
			for (size_t i = 0; i<settings_.N_; i++)
				this->getNonlinearSystemsInstances()[i] = typename Base::OptConProblem_t::DynamicsPtr_t(dyn->clone());
	}

	virtual void changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin) override
	{
		this->getLinearSystemsInstances().resize(settings_.N_);

		if(lin)
			for (size_t i = 0; i<settings_.N_; i++)
				this->getLinearSystemsInstances()[i] = typename Base::OptConProblem_t::LinearPtr_t(lin->clone());
	}

	virtual void changeIntermediateConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
	{
		this->getIntermediateConstraintsInstances().push_back(typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
	}

	virtual void changeFinalConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
	{
		this->getFinalConstraintsInstances().push_back(typename	Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
	}

	void printSolution()
	{
		dmsProblem_->printSolution();
	} 

private:
	std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM>> dmsProblem_;
	std::shared_ptr<NlpSolver> nlpSolver_;
	DmsSettings settings_;

	Policy_t policy_;

	state_vector_t x0_;
	core::Time tf_;
};	


} // namespace optcon
} // namespace ct

#endif // CT_OPTCON_DMS_SOLVER_HPP_