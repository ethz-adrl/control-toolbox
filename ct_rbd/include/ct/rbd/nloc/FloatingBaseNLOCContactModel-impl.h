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

namespace ct {
namespace rbd {

template <class RBDDynamics>
FloatingBaseNLOCContactModel<RBDDynamics>::FloatingBaseNLOCContactModel(const std::string& costFunctionFile,
	const std::string& settingsFile,
	std::shared_ptr<FBSystem> system,
	std::shared_ptr<LinearizedSystem> linearizedSystem)
	: system_(system),
	  linearizedSystem_(linearizedSystem),
	  costFunction_(new CostFunction(costFunctionFile, false)),
	  optConProblem_(system_, costFunction_, linearizedSystem_),
	  iteration_(0)
{
	solver_ = std::shared_ptr<NLOptConSolver>(new NLOptConSolver(optConProblem_, settingsFile));
}

template <class RBDDynamics>
FloatingBaseNLOCContactModel<RBDDynamics>::FloatingBaseNLOCContactModel(const std::string& costFunctionFile,
	const typename NLOptConSolver::Settings_t& settings,
	std::shared_ptr<FBSystem> system,
	std::shared_ptr<LinearizedSystem> linearizedSystem)
	: system_(system),
	  linearizedSystem_(linearizedSystem),
	  costFunction_(new CostFunction(costFunctionFile, false)),
	  optConProblem_(system_, costFunction_, linearizedSystem_),
	  iteration_(0)
{
	solver_ = std::shared_ptr<NLOptConSolver>(new NLOptConSolver(optConProblem_, settings));
}

template <class RBDDynamics>
void FloatingBaseNLOCContactModel<RBDDynamics>::initialize(const typename RBDDynamics::RBDState_t& x0,
	const core::Time& tf,
	StateVectorArray x_ref,
	FeedbackArray u0_fb,
	ControlVectorArray u0_ff)
{
	typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

	solver_->changeTimeHorizon(tf);
	solver_->setInitialGuess(policy);
	solver_->changeInitialState(x0.toStateVectorEulerXyz());
}

template <class RBDDynamics>
void FloatingBaseNLOCContactModel<RBDDynamics>::configure(const typename NLOptConSolver::Settings_t& settings)
{
	solver_->configure(settings);
}

template <class RBDDynamics>
bool FloatingBaseNLOCContactModel<RBDDynamics>::runIteration()
{
	bool foundBetter = solver_->runIteration();

	iteration_++;
	return foundBetter;
}

template <class RBDDynamics>
const typename FloatingBaseNLOCContactModel<RBDDynamics>::StateVectorArray&
FloatingBaseNLOCContactModel<RBDDynamics>::retrieveLastRollout()
{
	return solver_->getStates();
}

template <class RBDDynamics>
const typename FloatingBaseNLOCContactModel<RBDDynamics>::StateVectorArray&
FloatingBaseNLOCContactModel<RBDDynamics>::getStateVectorArray()
{
	return solver_->getSolution().x_ref();
}

template <class RBDDynamics>
const core::TimeArray& FloatingBaseNLOCContactModel<RBDDynamics>::getTimeArray()
{
	return solver_->getStateTrajectory().getTimeArray();
}

template <class RBDDynamics>
const typename FloatingBaseNLOCContactModel<RBDDynamics>::FeedbackArray&
FloatingBaseNLOCContactModel<RBDDynamics>::getFeedbackArray()
{
	return solver_->getSolution().K();
}

template <class RBDDynamics>
const typename FloatingBaseNLOCContactModel<RBDDynamics>::ControlVectorArray&
FloatingBaseNLOCContactModel<RBDDynamics>::getControlVectorArray()
{
	return solver_->getSolution().uff();
}

template <class RBDDynamics>
const typename FloatingBaseNLOCContactModel<RBDDynamics>::NLOptConSolver::Settings_t&
FloatingBaseNLOCContactModel<RBDDynamics>::getSettings() const
{
	return solver_->getSettings();
}

template <class RBDDynamics>
void FloatingBaseNLOCContactModel<RBDDynamics>::changeCostFunction(std::shared_ptr<CostFunction> costFunction)
{
	solver_->changeCostFunction(costFunction);
}

template <class RBDDynamics>
std::shared_ptr<typename FloatingBaseNLOCContactModel<RBDDynamics>::NLOptConSolver>
FloatingBaseNLOCContactModel<RBDDynamics>::getSolver()
{
	return solver_;
}
}
}
