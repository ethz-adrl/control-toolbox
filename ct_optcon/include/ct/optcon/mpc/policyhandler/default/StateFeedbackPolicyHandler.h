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
namespace optcon {

//! the default policy handler for iLQR
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class StateFeedbackPolicyHandler : public PolicyHandler<core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>,
									   STATE_DIM,
									   CONTROL_DIM,
									   SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> StateFeedbackController_t;

	StateFeedbackPolicyHandler(const SCALAR& dt);

	virtual ~StateFeedbackPolicyHandler();

	virtual void designWarmStartingPolicy(const SCALAR& delay,
		const SCALAR& newTimeHorizon,
		StateFeedbackController_t& policy) override;

	/*!
	 * required for additional post-truncation.
	 * @param delay 	the delay which is to be truncated away
	 * @param policy	the resulting, truncated policy
	 * @param effectivelyTruncated the time which was effectively truncated away
	 * 	(can be different from the input in discrete-time case, for example)
	 */
	virtual void truncateSolutionFront(const SCALAR& delay,
		StateFeedbackController_t& policy,
		SCALAR& effectivelyTruncated) override;

private:
	SCALAR dt_;
};

}  // namespace optcon
}  // namespace ct
