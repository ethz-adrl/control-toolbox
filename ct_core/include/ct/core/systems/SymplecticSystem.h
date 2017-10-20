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

#include "ControlledSystem.h"

namespace ct {
namespace core {

/**
 * @brief      The base class for the implementation of a symplectic system. In
 *             a symplectic system, the position and the velocity update can be
 *             separated. During integration, the velocity gets update first and
 *             the position update uses the updated velocity
 *
 * @tparam     POS_DIM      The position dimension
 * @tparam     VEL_DIM      The velocity dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The scalar type
 */
template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class SymplecticSystem : public ControlledSystem<POS_DIM + VEL_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef ControlledSystem<POS_DIM + VEL_DIM, CONTROL_DIM, SCALAR> Base;

	/**
	 * @brief      Constructor
	 *
	 * @param[in]  type  The type of the system
	 */
	SymplecticSystem(const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL) : Base(type) {}
	/**
	 * @brief      Constructor
	 *
	 * @param[in]  controller  The controller used when integrating the system
	 * @param[in]  type        The type of the system
	 */
	SymplecticSystem(std::shared_ptr<ct::core::Controller<POS_DIM + VEL_DIM, CONTROL_DIM, SCALAR>> controller,
		const SYSTEM_TYPE& type = SYSTEM_TYPE::GENERAL)
		: Base(controller, type)
	{
	}

	/**
	 * @brief      Copy constructor
	 *
	 * @param[in]  arg   The argument
	 */
	SymplecticSystem(const SymplecticSystem& arg) : ControlledSystem<POS_DIM + VEL_DIM, CONTROL_DIM, SCALAR>(arg) {}
	/**
	 * @brief      Destructor
	 */
	virtual ~SymplecticSystem() {}
	/**
	 * @brief      Creates a new instance of the object with same properties than original.
	 *
	 * @return     Copy of this object.
	 */
	virtual SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>* clone() const override = 0;

	virtual bool isSymplectic() const override { return true; }
	virtual void computeControlledDynamics(const StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
		const SCALAR& t,
		const ControlVector<CONTROL_DIM, SCALAR>& control,
		StateVector<POS_DIM + VEL_DIM, SCALAR>& derivative) override
	{
		StateVector<POS_DIM, SCALAR> pDot;
		StateVector<VEL_DIM, SCALAR> vDot;

		computePdot(state, state.tail(VEL_DIM), control, pDot);
		computeVdot(state, state.head(POS_DIM), control, vDot);

		derivative << pDot, vDot;
	}


	/**
	 * @brief      Computes the derivative of the position
	 *
	 * @param[in]  x     The full state vector
	 * @param[in]  v     The updated velocity
	 * @param[out] pDot  The derivative of the position
	 */
	void computePdot(const StateVector<POS_DIM + VEL_DIM, SCALAR>& x,
		const StateVector<VEL_DIM, SCALAR>& v,
		StateVector<POS_DIM, SCALAR>& pDot)
	{
		ControlVector<CONTROL_DIM, SCALAR> controlAction;
		if (this->controller_)
			this->controller_->computeControl(x, 0.0, controlAction);
		else
			controlAction.setZero();

		computePdot(x, v, controlAction, pDot);
	}

	/**
	 * @brief      Computes the derivative of the velocity
	 *
	 * @param[in]  x     The full state vector
	 * @param[in]  p     The position
	 * @param[out]      vDot  The derivative of the velocity
	 */
	void computeVdot(const StateVector<POS_DIM + VEL_DIM, SCALAR>& x,
		const StateVector<POS_DIM, SCALAR>& p,
		StateVector<VEL_DIM, SCALAR>& vDot)
	{
		ControlVector<CONTROL_DIM, SCALAR> controlAction;
		if (this->controller_)
			this->controller_->computeControl(x, 0.0, controlAction);
		else
			controlAction.setZero();

		computeVdot(x, p, controlAction, vDot);
	}


	/**
	 * @brief      Computes the derivative of the position
	 *
	 * @param[in]  x        The full state vector
	 * @param[in]  v        The updated velocity
	 * @param[in]  control  The control input
	 * @param[out] pDot     The derivative of the position
	 */
	virtual void computePdot(const StateVector<POS_DIM + VEL_DIM, SCALAR>& x,
		const StateVector<VEL_DIM, SCALAR>& v,
		const ControlVector<CONTROL_DIM, SCALAR>& control,
		StateVector<POS_DIM, SCALAR>& pDot) = 0;

	/**
	 * @brief      Computes the derivative of the velocity
	 *
	 * @param[in]  x        The full state vector
	 * @param[in]  p        The position
	 * @param[in]  control  The control input
	 * @param[out] vDot     The derivative of the velocity
	 */
	virtual void computeVdot(const StateVector<POS_DIM + VEL_DIM, SCALAR>& x,
		const StateVector<POS_DIM, SCALAR>& p,
		const ControlVector<CONTROL_DIM, SCALAR>& control,
		StateVector<VEL_DIM, SCALAR>& vDot) = 0;

protected:
};

}  // namespace core
}  // namespace ct
