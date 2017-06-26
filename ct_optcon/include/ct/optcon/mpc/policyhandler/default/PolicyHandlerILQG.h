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

#ifndef MPC_DEFAULT_POLICYHANDLER_ILQG_H_
#define MPC_DEFAULT_POLICYHANDLER_ILQG_H_

#include <ct/core/core.h>

// #define DEBUG_POLICYHANDLER_ILQG

namespace ct{
namespace optcon{

//! the default policy handler for SLQ
template<size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class PolicyHandlerILQG : public PolicyHandler<core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>, STATE_DIM, CONTROL_DIM, SCALAR>
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> StateFeedbackController_t;

	PolicyHandlerILQG(const SCALAR& dt):
		dt_ilqg_(dt)
	{}

	virtual ~PolicyHandlerILQG(){}


	virtual void designWarmStartingPolicy(
			const SCALAR& delay,
			const SCALAR& newTimeHorizon,
			StateFeedbackController_t& policy,
			core::StateTrajectory<STATE_DIM, SCALAR>& stateTraj) override {
		
		// get the current feedback and feedforward from the StateFeedbackController
		core::FeedbackTrajectory<STATE_DIM, CONTROL_DIM, SCALAR>& FeedbackTraj = policy.getFeedbackTrajectory();
		core::ControlTrajectory<CONTROL_DIM, SCALAR>& FeedForwardTraj = policy.getFeedforwardTrajectory();

		// current number of discrete elements
		int currentSize = FeedForwardTraj.size();

		// compute new controller length as a function of the time horizon
		int Kn_new = std::max(1, (int)std::lround(newTimeHorizon/dt_ilqg_));

		// compute number indices to be shifted. Note: it does not make sense to shift more entries than are available
		int num_di = FeedForwardTraj.getIndexFromTime(delay);
		num_di = std::min(num_di, currentSize-1);


#ifdef DEBUG_POLICYHANDLER_ILQG
		std::cout << "DEBUG_POLICYHANDLER_ILQG: Controller shifting: "<< std::endl <<
				"delay: " << delay << "  newT: " << newTimeHorizon << std::endl <<
				" new Discrete Controller has " << Kn_new << " elements, shifted about "
				<< num_di << " elements." << std::endl;
#endif


		// Step 1 - Truncate Front: remove first 'num_di' elements from controller and shift time accordingly
		if(num_di > 0)
		{
			FeedForwardTraj.eraseFront(num_di, num_di*dt_ilqg_);
			FeedbackTraj.eraseFront(num_di, num_di*dt_ilqg_);
			currentSize -=num_di;
		}


		// Step 2 - Resize overall controller
		if (Kn_new > currentSize)
		{
			//extend at back with constant value taken from last element
			bool timeIsRelative = true;
			for(size_t i = 0; i<Kn_new-currentSize; i++)
			{
				FeedbackTraj.push_back(FeedbackTraj.back(), dt_ilqg_, timeIsRelative);
				FeedForwardTraj.push_back(FeedForwardTraj.back(), dt_ilqg_, timeIsRelative);
			}
		}
		else if (Kn_new < currentSize)
		{
			// remove elements from back
			for(size_t i = 0; i< currentSize-Kn_new; i++)
			{
				FeedbackTraj.pop_back();
				FeedForwardTraj.pop_back();
			}
		}


		// safety check, which should never be entered
		if(FeedForwardTraj.size() == 0)
		{
			throw std::runtime_error("ERROR in PolicyHandlerILQG.h: new policy should not have size 0.");
		}
	}



	/*!
	 * required for additional post-truncation.
	 * @param delay 	the delay which is to be truncated away
	 * @param policy	the resulting, truncated policy
	 * @param stateTraj the state trajectory to be truncated
	 * @param effectivelyTruncated the time which was effectively truncated away
	 * 	(can be different from the input in discrete-time case, for example)
	 */
	virtual void truncateSolutionFront(
			const SCALAR& delay,
			StateFeedbackController_t& policy,
			core::StateTrajectory<STATE_DIM, SCALAR>& stateTraj,
			SCALAR& effectivelyTruncated) override {

		// current controller length
		size_t currentSize = policy.getFeedforwardTrajectory().size();

		size_t num_di = policy.getFeedforwardTrajectory().getIndexFromTime(delay);
		num_di = std::min(num_di, currentSize-1);

		effectivelyTruncated = num_di * dt_ilqg_;

#ifdef DEBUG_POLICYHANDLER_ILQG
		std::cout << "DEBUG_WARMSTART: Current Controller Size:  "<< currentSize << " elements." << std::endl;
		std::cout << "DEBUG_WARMSTART: Controller truncation: truncation about "<< num_di << " elements." << std::endl;
		std::cout << "DEBUG_WARMSTART: Controllernew size: "<< currentSize-num_di << " elements." << std::endl;
#endif

		// remove first num_di elements from controller
		if(num_di > 0 && num_di < currentSize)
		{
			policy.getFeedbackTrajectory().eraseFront (num_di, effectivelyTruncated);
			policy.getFeedforwardTrajectory().eraseFront (num_di, effectivelyTruncated);
			stateTraj.eraseFront(num_di, effectivelyTruncated);
		}
	}


private:

	SCALAR dt_ilqg_;

};

}	// namespace optcon
}	// namespace ct

#endif /* MPC_DEFAULT_POLICYHANDLER_ILQG_H_ */
