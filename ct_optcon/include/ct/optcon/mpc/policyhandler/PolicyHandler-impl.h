/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
PolicyHandler<POLICY, STATE_DIM, CONTROL_DIM, SCALAR>::PolicyHandler()
{
}

template <typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
PolicyHandler<POLICY, STATE_DIM, CONTROL_DIM, SCALAR>::~PolicyHandler()
{
}

template <typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void PolicyHandler<POLICY, STATE_DIM, CONTROL_DIM, SCALAR>::designWarmStartingPolicy(const SCALAR& delay,
    const SCALAR& TimeHorizon,
    POLICY& policy)
{
    policy = initialPolicy_;
}

template <typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void PolicyHandler<POLICY, STATE_DIM, CONTROL_DIM, SCALAR>::truncateSolutionFront(const SCALAR& delay,
    POLICY& policy,
    SCALAR& effectivelyTruncated)
{
}

template <typename POLICY, size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void PolicyHandler<POLICY, STATE_DIM, CONTROL_DIM, SCALAR>::setPolicy(const POLICY& newPolicy)
{
    initialPolicy_ = newPolicy;
}

}  // namespace optcon
}  // namespace ct
