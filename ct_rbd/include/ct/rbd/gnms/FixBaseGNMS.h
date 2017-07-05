/*
 * FixBaseGNMS.h
 *
 *  Created on: Jan 11, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_RBD_GNMS_FIXBASEGNMS_H_
#define INCLUDE_CT_RBD_GNMS_FIXBASEGNMS_H_

#include <ct/core/systems/linear/SystemLinearizer.h>

#include <ct/optcon/gnms/GNMS.hpp>

#include <ct/rbd/systems/FixBaseFDSystem.h>


namespace ct{
namespace rbd {

/**
 * \brief GNMS for fixed base systems without an explicit contact model.
 */
template <class RBDDynamics, typename GNMS_SCALAR = double>
class FixBaseGNMS
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const bool eeForcesAreControlInputs = false;

	typedef FixBaseFDSystem<RBDDynamics, eeForcesAreControlInputs> FBSystem;
	typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, GNMS_SCALAR> LinearizedSystem;
	typedef ct::rbd::RbdLinearizer<FBSystem> SystemLinearizer;

	typedef ct::optcon::GNMSBase<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, GNMS_SCALAR> GNMSBase;
	typedef ct::optcon::GNMS<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, GNMS_SCALAR> GNMS;

	typedef typename GNMS::StateVectorArray StateVectorArray;
	typedef typename GNMS::ControlVectorArray ControlVectorArray;

	typedef ct::optcon::CostFunctionAnalytical<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, GNMS_SCALAR> CostFunction;


	FixBaseGNMS(
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
		gnms_ = std::shared_ptr<GNMS>(new GNMS(optConProblem_, settingsFile));

	}

	void initialize(
			const core::Time& tf,
	            StateVectorArray x0 = StateVectorArray(),
	            ControlVectorArray u0_ff = ControlVectorArray())
	{
		typename GNMS::Policy_t policy(u0_ff, x0);

		gnms_->changeTimeHorizon(tf);
		gnms_->setInitialGuess(policy);
		gnms_->changeInitialState(x0[0]);
	}

	bool runIteration()
	{
		bool foundBetter = gnms_->runIteration();

		iteration_++;
		return foundBetter;
	}

	const StateVectorArray& retrieveLastRollout()
	{
		return gnms_->getStates();
	}

	const core::TimeArray& getTimeArray()
	{
		return gnms_->getStateTrajectory().getTimeArray();
	}

	const ControlVectorArray& getControlVectorArray()
	{
		return gnms_->getSolution().uff();
	}

	const typename GNMSBase::Settings_t& getSettings() const { return gnms_->getSettings(); }

	void changeCostFunction(std::shared_ptr<CostFunction> costFunction)
	{
		gnms_->changeCostFunction(costFunction);
	}

	std::shared_ptr<GNMSBase> getSolver()
	{
		return gnms_;
	}

private:

	std::shared_ptr<FBSystem> system_;
	std::shared_ptr<LinearizedSystem> linearizedSystem_;
	std::shared_ptr<CostFunction> costFunction_;

	optcon::OptConProblem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, GNMS_SCALAR> optConProblem_;

	std::shared_ptr<GNMSBase> gnms_;

	size_t iteration_;


};

}
}

#endif /* INCLUDE_CT_RBD_GNMS_FIXBASEGNMS_H_ */
