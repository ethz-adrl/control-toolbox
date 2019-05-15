/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {
namespace example {

/*!
 * @brief This method is called from different unit tests in order to compare the cost, first and second order gradients of two cost functions
 * @param costFunction the first cost function to be compared
 * @param costFunction2 the second cost function to be compared
 */
template <size_t state_dim, size_t control_dim>
void compareCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction,
    CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
    ASSERT_NEAR(costFunction.evaluateIntermediate(), costFunction2.evaluateIntermediate(), 1e-9);
    ASSERT_NEAR(costFunction.evaluateTerminal(), costFunction2.evaluateTerminal(), 1e-9);

    ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(costFunction2.stateDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateDerivativeTerminal().isApprox(costFunction2.stateDerivativeTerminal()));

    ASSERT_TRUE(
        costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction2.stateSecondDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateSecondDerivativeTerminal().isApprox(costFunction2.stateSecondDerivativeTerminal()));

    ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(costFunction2.controlDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.controlDerivativeTerminal().isApprox(costFunction2.controlDerivativeTerminal()));

    ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(
        costFunction2.controlSecondDerivativeIntermediate()));
    ASSERT_TRUE(
        costFunction.controlSecondDerivativeTerminal().isApprox(costFunction2.controlSecondDerivativeTerminal()));

    ASSERT_TRUE(
        costFunction.stateControlDerivativeIntermediate().isApprox(costFunction2.stateControlDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateControlDerivativeTerminal().isApprox(costFunction2.stateControlDerivativeTerminal()));

    // second derivatives have to be symmetric
    ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(
        costFunction.stateSecondDerivativeIntermediate().transpose()));
    ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(
        costFunction.controlSecondDerivativeIntermediate().transpose()));
}

}  // namespace example
}  // namesapce optcon
}  // namespace ct
