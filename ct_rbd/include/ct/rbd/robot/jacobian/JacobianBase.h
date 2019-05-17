/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \ingroup OS
 * @Brief Interface class for calculating Jacobain and its time derivative in an inertia frame (called as Origin frame).
 */
template <size_t NUM_OUTPUTS, size_t NUM_JOINTS, typename SCALAR>
class JacobianBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef RBDState<NUM_JOINTS, SCALAR> state_t;
    typedef Eigen::Matrix<SCALAR, NUM_OUTPUTS, 6 + NUM_JOINTS> jacobian_t;
    typedef Eigen::Matrix<SCALAR, 6 + NUM_JOINTS, NUM_OUTPUTS> jacobian_inv_t;

    JacobianBase(){};
    virtual ~JacobianBase(){};

    /**
	 * This methods calculates the Jacobian of the floating-base Jacobian in the Origin (World or Inertia)
	 * frame.
	 * @param state State of the RBD
	 * @param J     floating-base Jacobian in the Origin frame
	 */
    virtual void getJacobianOrigin(const state_t& state, jacobian_t& J) = 0;

    /**
	 * This methods calculates the time derivative of the Jacobian of the floating-base function in the
	 * Origin (World or Inertia) frame.
	 * @param state State of the RBD
	 * @param dJdt  Time derivative of the floating-base Jacobian in the Origin frame
	 */
    virtual void getJacobianOriginDerivative(const state_t& state, jacobian_t& dJdt) = 0;

private:
};

}  // namespace tpl

template <size_t NUM_OUTPUTS, size_t NUM_JOINTS>
using JacobianBase = tpl::JacobianBase<NUM_OUTPUTS, NUM_JOINTS, double>;

}  // namespace rbd
}  // namespace ct
