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

#ifndef INCLUDE_CT_RBD_NLOC_FLOATINGBASENLOCCONTACTMODEL_H_
#define INCLUDE_CT_RBD_NLOC_FLOATINGBASENLOCCONTACTMODEL_H_

#include <ct/optcon/optcon.h>

#include <ct/rbd/systems/FloatingBaseFDSystem.h>
#include <ct/rbd/systems/linear/RbdLinearizer.h>


namespace ct{
namespace rbd {

/**
 * \brief NLOC for floating base systems with an explicit contact model.
 */
template <class RBDDynamics>
class FloatingBaseNLOCContactModel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const bool quatIntegration = false;
	static const bool eeForcesAreControlInputs = false;

	typedef FloatingBaseFDSystem<RBDDynamics, false, false> FBSystem;
	typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> LinearizedSystem;
	typedef ct::rbd::RbdLinearizer<FBSystem> RBDLinearizer;
	typedef ct::core::SystemLinearizer<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> SystemLinearizer;

	typedef ct::optcon::NLOptConSolver<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> NLOptConSolver;

	typedef typename core::StateVector<FBSystem::STATE_DIM> StateVector;
	typedef typename core::ControlVector<FBSystem::CONTROL_DIM> ControlVector;
	typedef typename core::FeedbackMatrix<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> FeedbackMatrix;
	typedef typename core::StateVectorArray<FBSystem::STATE_DIM> StateVectorArray;
	typedef typename core::ControlVectorArray<FBSystem::CONTROL_DIM> ControlVectorArray;
	typedef typename core::FeedbackArray<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> FeedbackArray;

	typedef ct::optcon::CostFunctionAnalytical<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> CostFunction;


	//! constructor taking path to the settings file
	FloatingBaseNLOCContactModel(
			const std::string& costFunctionFile,
			const std::string& settingsFile,
			std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
			std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr
	) :
		system_(system),
		linearizedSystem_(linearizedSystem),
		costFunction_(new CostFunction(costFunctionFile, false)),
		optConProblem_(system_, costFunction_, linearizedSystem_),
		iteration_(0)
	{
			solver_ = std::shared_ptr<NLOptConSolver>(new NLOptConSolver(optConProblem_, settingsFile));
	}

	//! constructor taking settings file directly
	FloatingBaseNLOCContactModel(
			const std::string& costFunctionFile,
			const typename NLOptConSolver::Settings_t& settings,
			std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
			std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr
	) :
		system_(system),
		linearizedSystem_(linearizedSystem),
		costFunction_(new CostFunction(costFunctionFile, false)),
		optConProblem_(system_, costFunction_, linearizedSystem_),
		iteration_(0)
	{
			solver_ = std::shared_ptr<NLOptConSolver>(new NLOptConSolver(optConProblem_, settings));
	}

	void initialize(
			const typename RBDDynamics::RBDState_t& x0,
			const core::Time& tf,
			StateVectorArray x_ref = StateVectorArray(),
			FeedbackArray u0_fb = FeedbackArray(),
			ControlVectorArray u0_ff = ControlVectorArray())
	{
		typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

		solver_->changeTimeHorizon(tf);
		solver_->setInitialGuess(policy);
		solver_->changeInitialState(x0.toStateVectorEulerXyz());
	}

	void configure(const typename NLOptConSolver::Settings_t& settings) {solver_->configure(settings);}

	bool runIteration()
	{
		bool foundBetter = solver_->runIteration();

		iteration_++;
		return foundBetter;
	}

	const StateVectorArray& retrieveLastRollout()
	{
		return solver_->getStates();
	}

	const StateVectorArray& getStateVectorArray()
	{
		return solver_->getSolution().x_ref();
	}

	const core::TimeArray& getTimeArray()
	{
		return solver_->getStateTrajectory().getTimeArray();
	}

	const FeedbackArray& getFeedbackArray()
	{
		return solver_->getSolution().K();
	}

	const ControlVectorArray& getControlVectorArray()
	{
		return solver_->getSolution().uff();
	}		

	const typename NLOptConSolver::Settings_t& getSettings() const { return solver_->getSettings(); }

	void changeCostFunction(std::shared_ptr<CostFunction> costFunction)
	{
		solver_->changeCostFunction(costFunction);
	}

	std::shared_ptr<NLOptConSolver> getSolver()
	{
		return solver_;
	}

private:

	std::shared_ptr<FBSystem> system_;
	std::shared_ptr<LinearizedSystem> linearizedSystem_;
	std::shared_ptr<CostFunction> costFunction_;

	optcon::OptConProblem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> optConProblem_;

	std::shared_ptr<NLOptConSolver> solver_;

	size_t iteration_;


};

}
}

#endif /* INCLUDE_CT_RBD_NLOC_FLOATINGBASENLOCCONTACTMODEL_H_ */
