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

#ifndef CT_OPTCON_DMS_DMS_CORE_SOLVER_H_
#define CT_OPTCON_DMS_DMS_CORE_SOLVER_H_

#include <ct/optcon/problem/OptConProblem.h>
#include <ct/optcon/solver/OptConSolver.h>

#include <ct/optcon/dms/dms_core/DmsProblem.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>

#include <ct/optcon/nlp/Nlp>

#include <memory>

namespace ct {
namespace optcon {

/** @defgroup   DMS DMS
 *
 * @brief      The direct multiple shooting module
 */


/**
 * @ingroup    DMS
 *
 * @brief      The DMS policy used as a solution container 
 *
 */
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


/**
 * @ingroup    DMS
 *
 * @brief      Class to solve a specfic DMS problem
 *
 * An example employing different DMS solvers is given in unit test \ref oscDMSTest.cpp
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
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

	/**
	 * @brief      Custom constructor, converts the optcon problem to a DMS problem
	 *
	 * @param[in]  problem      The optimal control problem	
	 * @param[in]  settingsDms  The dms settings
	 */
	DmsSolver(const OptConProblem<STATE_DIM, CONTROL_DIM> problem, DmsSettings settingsDms) 
	:
	nlpSolver_(nullptr),
	settings_(settingsDms)
	{
		// Create system, linearsystem and costfunction instances 
		this->setProblem(problem);

		dmsProblem_ = std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM>> (new DmsProblem<STATE_DIM, CONTROL_DIM>
				(settingsDms, this->systems_, this->linearSystems_, this->costFunctions_, 
					this->stateInputConstraints_, this->pureStateConstraints_, x0_));

		if(settingsDms.nlpSettings_.solverType_ == NlpSolverSettings::SNOPT)
			nlpSolver_ = std::shared_ptr<SnoptSolver>(new SnoptSolver(dmsProblem_, settingsDms.nlpSettings_));
		else if (settingsDms.nlpSettings_.solverType_ == NlpSolverSettings::IPOPT)
			nlpSolver_ = std::shared_ptr<IpoptSolver> (new IpoptSolver(dmsProblem_, settingsDms.nlpSettings_));
		else
			std::cout << "Unknown solver type... Exiting" << std::endl;

		configure(settingsDms);
	}

	/**
	 * @brief      Destructor
	 */
	virtual ~DmsSolver(){}

	virtual void configure(const DmsSettings& settings) override
	{
		dmsProblem_->configure(settings);
		dmsProblem_->changeTimeHorizon(tf_);
		dmsProblem_->changeInitialState(x0_);
		nlpSolver_->configure(settings_.nlpSettings_); 
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
		if(dmsProblem_)
			dmsProblem_->changeTimeHorizon(tf);
	}

	virtual void changeInitialState(const core::StateVector<STATE_DIM>& x0) override
	{
		x0_ = x0;
		if(dmsProblem_)
			dmsProblem_->changeInitialState(x0);
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

	virtual void changeStateInputConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
	{
		this->getStateInputConstraintsInstances().push_back(typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
	}

	virtual void changePureStateConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
	{
		this->getPureStateConstraintsInstances().push_back(typename	Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
	}

	/**
	 * @brief      Prints out the solution trajectories of the DMS problem
	 */
	void printSolution()
	{
		dmsProblem_->printSolution();
	} 

private:
	std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM>> dmsProblem_; /*!<The dms problem*/
	std::shared_ptr<NlpSolver> nlpSolver_; /*!<The nlp solver for solving the dmsproblem*/
	DmsSettings settings_; /*!<The dms settings*/

	Policy_t policy_; /*!<The solution container*/
 
	state_vector_t x0_; /*!<The initial state for the optimization*/
	core::Time tf_; /*!<The timehorizon of the problem*/
};	


} // namespace optcon
} // namespace ct

#endif // CT_OPTCON_DMS_SOLVER_H_
