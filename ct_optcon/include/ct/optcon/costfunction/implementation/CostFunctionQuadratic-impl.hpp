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


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediateNumDiff()
{
	state_vector_t dFdx = state_vector_t::Zero();
	state_vector_t x_local;
	control_vector_t u_local; 
	SCALAR t_local;
	this->getCurrentStateAndControl(x_local, u_local, t_local);
	SCALAR dxdt_ref = this->evaluateIntermediate();

	for (size_t i=0; i < STATE_DIM; ++i)
	{
		// inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
		SCALAR h = eps_ * std::max(std::abs(x_local(i)), 1.0);
		volatile SCALAR x_ph = x_local(i) + h;
		SCALAR dxp = x_ph - x_local(i);

		state_vector_t x_perturbed = x_local;
		x_perturbed(i) =  x_ph;

		// get evaluation of f(x,u)
		this->setCurrentStateAndControl(x_perturbed, u_local, t_local);
		SCALAR dxdt = this->evaluateIntermediate();

		if (doubleSidedDerivative_)
		{
			SCALAR dxdt_low;

			volatile SCALAR x_mh = x_local(i) - h;
			SCALAR dxm = x_local(i) - x_mh;

			x_perturbed = x_local;
			x_perturbed(i) = x_mh;
			this->setCurrentStateAndControl(x_perturbed, u_local, t_local);
			dxdt_low = this->evaluateIntermediate();
			dFdx(i,0) = (dxdt - dxdt_low) / (dxp + dxm);
		}
		else
		{
			dFdx(i,0) = (dxdt - dxdt_ref) / dxp;
		}
	}

	return dFdx;		
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediateNumDiff()
{
	control_vector_t dFdu = control_vector_t::Zero();
	state_vector_t x_local;
	control_vector_t u_local; 
	SCALAR t_local;
	this->getCurrentStateAndControl(x_local, u_local, t_local);
	SCALAR dxdt_ref = this->evaluateIntermediate();

	for (size_t i=0; i < CONTROL_DIM; ++i)
	{
		// inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
		SCALAR h = eps_ * std::max(std::abs(u_local(i)), 1.0);
		volatile SCALAR u_ph = u_local(i) + h;
		SCALAR dup = u_ph - u_local(i);

		control_vector_t u_perturbed = u_local;
		u_perturbed(i) =  u_ph;

		// get evaluation of f(x,u)
		this->setCurrentStateAndControl(x_local, u_perturbed, t_local);
		SCALAR dxdt = this->evaluateIntermediate();

		if (doubleSidedDerivative_)
		{
			SCALAR dxdt_low;

			volatile SCALAR u_mh = u_local(i) - h;
			SCALAR dum = u_local(i) - u_mh;

			u_perturbed = u_local;
			u_perturbed(i) = u_mh;
			this->setCurrentStateAndControl(x_local, u_perturbed, t_local);
			dxdt_low = this->evaluateIntermediate();

			dFdu(i,0) = (dxdt - dxdt_low) / (dup + dum);
		}
		else
		{
			dFdu(i,0) = (dxdt - dxdt_ref) / dup;
		}
	}

	return dFdu;
}
