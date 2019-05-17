/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class PolicyHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolicyHandler();

    virtual ~PolicyHandler();

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
    virtual void designWarmStartingPolicy(const SCALAR& delay, const SCALAR& TimeHorizon, POLICY& policy);


    //! a method required for additional post-truncation.
    /*!
	 * post truncation may become relevant if the delay is underestimated or pre-integration is turned off.
	 * @param delay
	 * 	the time to truncate away from the solution
	 * @param policy
	 *  the policy to be truncated
	 * @param effectivelyTruncated
	 * the time which was truncated away
	 */
    virtual void truncateSolutionFront(const SCALAR& delay, POLICY& policy, SCALAR& effectivelyTruncated);


    //! set new policy to policy handler
    void setPolicy(const POLICY& newPolicy);


protected:
    POLICY initialPolicy_;  //! the initial policy
};

}  // namespace optcon
}  // namespace ct
