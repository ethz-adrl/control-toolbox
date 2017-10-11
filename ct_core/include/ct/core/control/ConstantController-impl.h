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

#ifndef CT_CORE_CONSTANT_CONTROLLER_IMPL_H_
#define CT_CORE_CONSTANT_CONTROLLER_IMPL_H_

namespace ct {
namespace core {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::ConstantController()
{
	u_.setZero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::ConstantController(ControlVector<CONTROL_DIM, SCALAR>& u):
	u_(u)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::ConstantController(const ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>& other) :
	Controller<STATE_DIM, CONTROL_DIM, SCALAR>(other),
	u_(other.u_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::~ConstantController() {}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>* ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
	return new ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::computeControl(
			const StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& t,
			ControlVector<CONTROL_DIM, SCALAR>& controlAction)
{
	controlAction = u_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::setControl(const ControlVector<CONTROL_DIM, SCALAR>& u)
{
	u_ = u;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
const ControlVector<CONTROL_DIM, SCALAR>& ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::getControl() const
{
	return u_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ControlMatrix<CONTROL_DIM, SCALAR> ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>::getDerivativeU0(const StateVector<STATE_DIM, SCALAR>& state, const SCALAR time)
{
	return ControlMatrix<CONTROL_DIM, SCALAR>::Identity();
}

}
}

#endif /* CT_CORE_CONSTANT_CONTROLLER_IMPL_H_ */
