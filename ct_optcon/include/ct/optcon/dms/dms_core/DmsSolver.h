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

#pragma once

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
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
struct DmsPolicy
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
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
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DmsSolver : public OptConSolver<DmsSolver<STATE_DIM, CONTROL_DIM, SCALAR>,
					  DmsPolicy<STATE_DIM, CONTROL_DIM, SCALAR>,
					  DmsSettings,
					  STATE_DIM,
					  CONTROL_DIM,
					  SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef OptConSolver<DmsSolver<STATE_DIM, CONTROL_DIM, SCALAR>,
		DmsPolicy<STATE_DIM, CONTROL_DIM, SCALAR>,
		DmsSettings,
		STATE_DIM,
		CONTROL_DIM>
		Base;

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef DmsPolicy<STATE_DIM, CONTROL_DIM, SCALAR> Policy_t;

	typedef OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;

	/**
	 * @brief      Custom constructor, converts the optcon problem to a DMS problem
	 *
	 * @param[in]  problem      The optimal control problem	
	 * @param[in]  settingsDms  The dms settings
	 */
	DmsSolver(const OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> problem, DmsSettings settingsDms)
		: nlpSolver_(nullptr), settings_(settingsDms)
	{
		// Create system, linearsystem and costfunction instances
		this->setProblem(problem);

		dmsProblem_ = std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM, SCALAR>>(
			new DmsProblem<STATE_DIM, CONTROL_DIM, SCALAR>(settingsDms, this->systems_, this->linearSystems_,
				this->costFunctions_, this->stateInputConstraints_, this->pureStateConstraints_, x0_));

		// SNOPT only works for the double type
		if (settingsDms.solverSettings_.solverType_ == NlpSolverSettings::SNOPT)
			nlpSolver_ = std::shared_ptr<SnoptSolver>(new SnoptSolver(dmsProblem_, settingsDms.solverSettings_));
		else if (settingsDms.solverSettings_.solverType_ == NlpSolverSettings::IPOPT)
			nlpSolver_ = std::shared_ptr<IpoptSolver>(new IpoptSolver(dmsProblem_, settingsDms.solverSettings_));
		else
			std::cout << "Unknown solver type... Exiting" << std::endl;

		configure(settingsDms);
	}

	virtual void generateAndCompileCode(const OptConProblem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>& problemCG,
		const ct::core::DerivativesCppadSettings& settings) override
	{
		// Create system, linearsystem and costfunction instances
		typedef std::shared_ptr<core::ControlledSystem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>> SysPtrCG;
		typedef std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>> LinearSysPtrCG;
		typedef std::shared_ptr<optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>> CostPtrCG;
		typedef std::shared_ptr<optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>
			ConstraintCG;
		ct::core::StateVector<STATE_DIM, ct::core::ADCGScalar> x0CG = problemCG.getInitialState();

		std::vector<SysPtrCG> systemsCG;
		std::vector<LinearSysPtrCG> linearSystemsCG;
		std::vector<CostPtrCG> costFunctionsCG;
		std::vector<ConstraintCG> stateInputConstraintsCG;
		std::vector<ConstraintCG> pureStateConstraintsCG;

		for (size_t i = 0; i < settings_.N_; i++)
		{
			systemsCG.push_back(SysPtrCG(problemCG.getNonlinearSystem()->clone()));
			linearSystemsCG.push_back(LinearSysPtrCG(problemCG.getLinearSystem()->clone()));
			costFunctionsCG.push_back(CostPtrCG(problemCG.getCostFunction()->clone()));
		}

		if (problemCG.getStateInputConstraints())
			stateInputConstraintsCG.push_back(ConstraintCG(problemCG.getStateInputConstraints()->clone()));

		if (problemCG.getPureStateConstraints())
			pureStateConstraintsCG.push_back(ConstraintCG(problemCG.getPureStateConstraints()->clone()));

		dmsProblem_->generateAndCompileCode(
			systemsCG, linearSystemsCG, costFunctionsCG, stateInputConstraintsCG, pureStateConstraintsCG, x0CG);
	}

	/**
	 * @brief      Destructor
	 */
	virtual ~DmsSolver() {}
	virtual void configure(const DmsSettings& settings) override
	{
		dmsProblem_->configure(settings);
		dmsProblem_->changeTimeHorizon(tf_);
		dmsProblem_->changeInitialState(x0_);
	}

	virtual bool solve() override
	{
		if (!nlpSolver_->isInitialized())
			nlpSolver_->configure(settings_.solverSettings_);

		return nlpSolver_->solve();
	}

	virtual const Policy_t& getSolution() override
	{
		policy_.xSolution_ = dmsProblem_->getStateSolution();
		policy_.uSolution_ = dmsProblem_->getInputSolution();
		policy_.tSolution_ = dmsProblem_->getTimeSolution();
		return policy_;
	}

	virtual const core::StateTrajectory<STATE_DIM, SCALAR> getStateTrajectory() const override
	{
		return core::StateTrajectory<STATE_DIM, SCALAR>(dmsProblem_->getTimeArray(), dmsProblem_->getStateTrajectory());
	}

	virtual const core::ControlTrajectory<CONTROL_DIM, SCALAR> getControlTrajectory() const override
	{
		return core::ControlTrajectory<CONTROL_DIM, SCALAR>(
			dmsProblem_->getTimeArray(), dmsProblem_->getInputTrajectory());
	}

	virtual const core::tpl::TimeArray<SCALAR>& getTimeArray() const override { return dmsProblem_->getTimeArray(); }
	virtual void setInitialGuess(const Policy_t& initialGuess) override
	{
		dmsProblem_->setInitialGuess(initialGuess.xSolution_, initialGuess.uSolution_);
	}

	virtual SCALAR getTimeHorizon() const override { return dmsProblem_->getTimeHorizon(); }
	virtual void changeTimeHorizon(const SCALAR& tf) override
	{
		tf_ = tf;
		if (dmsProblem_)
			dmsProblem_->changeTimeHorizon(tf);
	}

	virtual void changeInitialState(const core::StateVector<STATE_DIM, SCALAR>& x0) override
	{
		x0_ = x0;
		if (dmsProblem_)
			dmsProblem_->changeInitialState(x0);
	}

	virtual void changeCostFunction(const typename Base::OptConProblem_t::CostFunctionPtr_t& cf) override
	{
		this->getCostFunctionInstances().resize(settings_.N_);
		if (cf)
			for (size_t i = 0; i < settings_.N_; i++)
				this->getCostFunctionInstances()[i] = typename Base::OptConProblem_t::CostFunctionPtr_t(cf->clone());
	}

	virtual void changeNonlinearSystem(const typename Base::OptConProblem_t::DynamicsPtr_t& dyn) override
	{
		this->getNonlinearSystemsInstances().resize(settings_.N_);

		if (dyn)
			for (size_t i = 0; i < settings_.N_; i++)
				this->getNonlinearSystemsInstances()[i] = typename Base::OptConProblem_t::DynamicsPtr_t(dyn->clone());
	}

	virtual void changeLinearSystem(const typename Base::OptConProblem_t::LinearPtr_t& lin) override
	{
		this->getLinearSystemsInstances().resize(settings_.N_);

		if (lin)
			for (size_t i = 0; i < settings_.N_; i++)
				this->getLinearSystemsInstances()[i] = typename Base::OptConProblem_t::LinearPtr_t(lin->clone());
	}

	virtual void changeStateInputConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
	{
		this->getStateInputConstraintsInstances().push_back(
			typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
	}

	virtual void changePureStateConstraints(const typename Base::OptConProblem_t::ConstraintPtr_t con) override
	{
		this->getPureStateConstraintsInstances().push_back(
			typename Base::OptConProblem_t::ConstraintPtr_t(con->clone()));
	}

	/**
	 * @brief      Prints out the solution trajectories of the DMS problem
	 */
	void printSolution() { dmsProblem_->printSolution(); }
	virtual std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() override
	{
		return systems_;
	}
	virtual const std::vector<typename OptConProblem_t::DynamicsPtr_t>& getNonlinearSystemsInstances() const override
	{
		return systems_;
	}

	virtual std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() override
	{
		return linearSystems_;
	}
	virtual const std::vector<typename OptConProblem_t::LinearPtr_t>& getLinearSystemsInstances() const override
	{
		return linearSystems_;
	}

	virtual std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() override
	{
		return costFunctions_;
	}
	virtual const std::vector<typename OptConProblem_t::CostFunctionPtr_t>& getCostFunctionInstances() const override
	{
		return costFunctions_;
	}

	virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances() override
	{
		return stateInputConstraints_;
	}
	virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getStateInputConstraintsInstances()
		const override
	{
		return stateInputConstraints_;
	}

	virtual std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances() override
	{
		return pureStateConstraints_;
	}
	virtual const std::vector<typename OptConProblem_t::ConstraintPtr_t>& getPureStateConstraintsInstances()
		const override
	{
		return pureStateConstraints_;
	}


private:
	std::shared_ptr<DmsProblem<STATE_DIM, CONTROL_DIM, SCALAR>> dmsProblem_; /*!<The dms problem*/
	std::shared_ptr<tpl::NlpSolver<SCALAR>> nlpSolver_; /*!<The nlp solver for solving the dmsproblem*/
	DmsSettings settings_;                              /*!<The dms settings*/

	Policy_t policy_; /*!<The solution container*/

	state_vector_t x0_; /*!<The initial state for the optimization*/
	SCALAR tf_;         /*!<The timehorizon of the problem*/

	std::vector<typename OptConProblem_t::DynamicsPtr_t> systems_;
	std::vector<typename OptConProblem_t::LinearPtr_t> linearSystems_;
	std::vector<typename OptConProblem_t::CostFunctionPtr_t> costFunctions_;
	std::vector<typename OptConProblem_t::ConstraintPtr_t> stateInputConstraints_;
	std::vector<typename OptConProblem_t::ConstraintPtr_t> pureStateConstraints_;
};


}  // namespace optcon
}  // namespace ct
