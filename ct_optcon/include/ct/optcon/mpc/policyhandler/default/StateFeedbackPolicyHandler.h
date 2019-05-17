/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

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
