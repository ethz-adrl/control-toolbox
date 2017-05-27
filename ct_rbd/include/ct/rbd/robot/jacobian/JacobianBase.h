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

#ifndef CT_JACOBIANSBASE_H_
#define CT_JACOBIANSBASE_H_

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
	typedef RBDState<NUM_JOINTS, SCALAR>		 				state_t;
	typedef Eigen::Matrix<SCALAR, NUM_OUTPUTS, 6+NUM_JOINTS> 	jacobian_t;
	typedef Eigen::Matrix<SCALAR, 6+NUM_JOINTS, NUM_OUTPUTS> 	jacobian_inv_t;

	JacobianBase() {};
	virtual ~JacobianBase() {};

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

} // namespace tpl

template<size_t NUM_OUTPUTS, size_t NUM_JOINTS>
using JacobianBase = tpl::JacobianBase<NUM_OUTPUTS, NUM_JOINTS, double>;

} // namespace rbd
} // namespace ct

#endif /* CT_JACOBIANSBASE_H_ */
