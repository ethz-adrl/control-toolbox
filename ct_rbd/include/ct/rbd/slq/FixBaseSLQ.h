/*
 * FixBaseSLQ.h
 *
 *  Created on: Jan 11, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_RBD_SLQ_FIXBASESLQ_H_
#define INCLUDE_CT_RBD_SLQ_FIXBASESLQ_H_

#include <ct/core/systems/linear/SystemLinearizer.h>

#include <ct/optcon/ilqg/iLQGMP.hpp>

#include <ct/rbd/systems/FixBaseFDSystem.h>


namespace ct{
namespace rbd {

/**
 * \brief SLQ for fixed base systems without an explicit contact model.
 */
template <class RBDDynamics, typename ILQG_SCALAR = double>
class FixBaseSLQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const bool eeForcesAreControlInputs = false;

	typedef FixBaseFDSystem<RBDDynamics, eeForcesAreControlInputs> FBSystem;
	typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, ILQG_SCALAR> LinearizedSystem;
	typedef ct::rbd::RbdLinearizer<FBSystem> SystemLinearizer;

	typedef ct::optcon::iLQGBase<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, ILQG_SCALAR> iLQGBase;
	typedef ct::optcon::iLQG<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, ILQG_SCALAR> iLQG;
	typedef ct::optcon::iLQGMP<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, ILQG_SCALAR> iLQGMP;

	typedef typename iLQG::StateVectorArray StateVectorArray;
	typedef typename iLQG::FeedbackArray FeedbackArray;
	typedef typename iLQG::ControlVectorArray ControlVectorArray;

	typedef ct::optcon::CostFunctionAnalytical<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, ILQG_SCALAR> CostFunction;


	FixBaseSLQ(
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
			const tpl::JointState<FBSystem::CONTROL_DIM, ILQG_SCALAR>& x0,
			const core::Time& tf,
	            FeedbackArray u0_fb = FeedbackArray(),
	            ControlVectorArray u0_ff = ControlVectorArray())
	{
		typename iLQG::Policy_t policy(u0_ff, u0_fb, getSettings().dt);

		ilqg_->changeTimeHorizon(tf);
		ilqg_->setInitialGuess(policy);
		ilqg_->changeInitialState(x0.toImplementation());
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

	optcon::OptConProblem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, ILQG_SCALAR> optConProblem_;

	std::shared_ptr<iLQGBase> ilqg_;

	size_t iteration_;


};

}
}

#endif /* INCLUDE_CT_RBD_SLQ_FIXBASESLQ_H_ */
