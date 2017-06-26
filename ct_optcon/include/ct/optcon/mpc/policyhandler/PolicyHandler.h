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

#ifndef MPC_POLICYHANDLER_H_
#define MPC_POLICYHANDLER_H_

namespace ct{
namespace optcon{

template<typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class PolicyHandler{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PolicyHandler(){}

	virtual ~PolicyHandler(){}

	//! design a warm-starting policy for the optimal control problem solver
	/*!
	 * Designs an initial guess for MPC (warm-start). An optimal strategy for warm-starting might be highly application/system dependent,
	 * thus the user can overload this method if desired. Straight-forward default implementations for common 'ct' solvers and policy
	 * types are provided in the folder "default".
	 * Note that the default policy handler simply performs "cold-starting" which means that the initially provided control policy is returned without modification.
	 *
	 * @param delay
	 * 	time difference between nominal starting time of the current policy and when the warm-start policy should start
	 * @param TimeHorizon
	 * 	desired overall policy time horizon (note: not covering the whole time-horizon may result in an error)
	 * @param policy
	 * 	the current policy, to be overwritten with the warm start
	 */
	virtual void designWarmStartingPolicy(
			const SCALAR& delay,
			const SCALAR& TimeHorizon,
			POLICY& policy,
			core::StateTrajectory<STATE_DIM, SCALAR>& stateTraj)
	{
		policy = initialPolicy_;
	}


	//! a method required for additional post-truncation.
	/*!
	 * post truncation may become relevant if the delay is underestimated or pre-integration is turned off.
	 * @param delay
	 * 	the time to truncate away from the solution
	 * @param policy
	 *  the policy to be truncated
	 * @param traj
	 *  the state trajectory to be truncated
	 * @param effectivelyTruncated
	 * the time which was truncated away
	 */
	virtual void truncateSolutionFront(
			const SCALAR& delay,
			POLICY& policy,
			core::StateTrajectory<STATE_DIM, SCALAR>& traj,
			SCALAR& effectivelyTruncated) {}


	//! set new policy to policy handler
	void setPolicy(const POLICY& newPolicy) {initialPolicy_ = newPolicy;}


protected:

	POLICY initialPolicy_;	//! the initial policy

};

}	// namespace optcon
}	// namespace ct



#endif /* MPC_POLICYHANDLER_H_ */
