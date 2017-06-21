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

#ifndef INCLUDE_CT_RBD_SLQ_FLOATINGBASESLQCONTACTMODEL_H_
#define INCLUDE_CT_RBD_SLQ_FLOATINGBASESLQCONTACTMODEL_H_

#include <ct/core/systems/linear/SystemLinearizer.h>

#include <ct/optcon/optcon.h>

#include <ct/rbd/systems/FloatingBaseFDSystem.h>
#include <ct/rbd/systems/linear/RbdLinearizer.h>


namespace ct{
namespace rbd {

/**
 * \brief SLQ for floating base systems with an explicit contact model.
 */
template <class RBDDynamics>
class FloatingBaseSLQContactModel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const bool quatIntegration = false;
	static const bool eeForcesAreControlInputs = false;

	typedef FloatingBaseFDSystem<RBDDynamics, false, false> FBSystem;
	typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> LinearizedSystem;
	typedef ct::rbd::RbdLinearizer<FBSystem> RBDLinearizer;
	typedef ct::core::SystemLinearizer<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> SystemLinearizer;

	typedef ct::optcon::iLQGBase<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> iLQGBase;
	typedef ct::optcon::iLQG<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> iLQG;
	typedef ct::optcon::iLQGMP<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> iLQGMP;

	typedef typename iLQG::StateVectorArray StateVectorArray;
	typedef typename iLQG::FeedbackArray FeedbackArray;
	typedef typename iLQG::ControlVectorArray ControlVectorArray;

	typedef ct::optcon::CostFunctionAnalytical<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> CostFunction;


	FloatingBaseSLQContactModel(
			const std::string& costFunctionFile,
			const std::string& settingsFile,
			std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
			std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr,
			bool useMP = true
	) :
		system_(system),
		linearizedSystem_(linearizedSystem),
		costFunction_(new CostFunction(costFunctionFile, false)),
		optConProblem_(system_, costFunction_, linearizedSystem_),
		iteration_(0)
	{
		if (useMP)
			ilqg_ = std::shared_ptr<iLQGMP>(new iLQGMP(optConProblem_, settingsFile));
		else
			ilqg_ = std::shared_ptr<iLQG>(new iLQG(optConProblem_, settingsFile));

	}

	void initialize(
			const typename RBDDynamics::RBDState_t& x0,
			const core::Time& tf,
	            FeedbackArray u0_fb = FeedbackArray(),
	            ControlVectorArray u0_ff = ControlVectorArray())
	{
		typename iLQG::Policy_t policy(u0_ff, u0_fb, getSettings().dt);

		ilqg_->changeTimeHorizon(tf);
		ilqg_->setInitialGuess(policy);
		ilqg_->changeInitialState(x0.toStateVectorEulerXyz());
	}

	bool runIteration()
	{
		bool foundBetter = ilqg_->runIteration();

		iteration_++;
		return foundBetter;
	}

	const StateVectorArray& retrieveLastRollout()
	{
		return ilqg_->getStates();
	}

	const core::TimeArray& getTimeArray()
	{
		return ilqg_->getStateTrajectory().getTimeArray();
	}

	const FeedbackArray& getFeedbackArray()
	{
		return ilqg_->getSolution().K();
	}

	const ControlVectorArray& getControlVectorArray()
	{
		return ilqg_->getSolution().uff();
	}		

	const typename iLQGBase::Settings_t& getSettings() const { return ilqg_->getSettings(); }

	void changeCostFunction(std::shared_ptr<CostFunction> costFunction)
	{
		ilqg_->changeCostFunction(costFunction);
	}

	std::shared_ptr<iLQGBase> getSolver()
	{
		return ilqg_;
	}

private:

	std::shared_ptr<FBSystem> system_;
	std::shared_ptr<LinearizedSystem> linearizedSystem_;
	std::shared_ptr<CostFunction> costFunction_;

	optcon::OptConProblem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> optConProblem_;

	std::shared_ptr<iLQGBase> ilqg_;

	size_t iteration_;


};

}
}

#endif /* INCLUDE_CT_RBD_SLQ_FLOATINGBASESLQCONTACTMODEL_H_ */