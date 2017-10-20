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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#include <ct/core/integration/EventHandler.h>

namespace ct {
namespace core {

//! Event handler to record substeps
/*!
 * This event handler records subintegration steps
 *
 * @tparam STATE_DIM size of the state vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SubstepRecorder : public EventHandler<STATE_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef StateVector<STATE_DIM, SCALAR> state_t;

	SubstepRecorder(std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system = nullptr,
		bool activated = true,
		std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> states =
			std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>(
				new ct::core::StateVectorArray<STATE_DIM, SCALAR>),
		std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>> controls =
			std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>(
				new ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>),
		std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>> times = std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>>(
			new ct::core::tpl::TimeArray<SCALAR>))
		: activated_(activated), system_(system), states_(states), controls_(controls), times_(times)
	{
	}

	//! default destructor
	virtual ~SubstepRecorder() {}
	virtual bool callOnSubsteps() override { return true; }
	void setControlledSystem(const std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& system)
	{
		system_ = system;
	}

	//! checks the kill flag
	virtual bool checkEvent(const state_t& state, const SCALAR& t) override { return activated_; }
	//! records the state
	virtual void handleEvent(const state_t& state, const SCALAR& t) override
	{
		states_->push_back(state);
		controls_->push_back(system_->getLastControlAction());
		times_->push_back(t);
	}

	void setEnable(bool activated) { activated_ = activated; }
	//! resets kill flag to false
	virtual void reset() override
	{
		states_ = std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>(
			new ct::core::StateVectorArray<STATE_DIM, SCALAR>);
		controls_ = std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>(
			new ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>);
		times_ = std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>>(new ct::core::tpl::TimeArray<SCALAR>);
	};

	const std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>>& getSubstates() const { return states_; }
	const std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>> getSubcontrols() const
	{
		return controls_;
	}  //!< container for logging the control

private:
	bool activated_;

	std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system_;

	std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> states_;  //!< container for logging the state
	std::shared_ptr<ct::core::ControlVectorArray<CONTROL_DIM, SCALAR>>
		controls_;                                             //!< container for logging the control
	std::shared_ptr<ct::core::tpl::TimeArray<SCALAR>> times_;  //!< container for logging the time
};
}
}
