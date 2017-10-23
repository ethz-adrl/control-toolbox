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

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerBase()
	: x_(state_vector_t::Zero()), u_(input_vector_t::Zero()), t_(SCALAR(0.0))
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerBase(const ConstraintContainerBase& arg)
	: x_(arg.x_),
	  u_(arg.u_),
	  t_(arg.t_),
	  lowerBoundsIntermediate_(arg.lowerBoundsIntermediate_),
	  lowerBoundsTerminal_(arg.lowerBoundsTerminal_),
	  upperBoundsIntermediate_(arg.upperBoundsIntermediate_),
	  upperBoundsTerminal_(arg.upperBoundsTerminal_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::~ConstraintContainerBase()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
	const input_vector_t& u,
	const SCALAR t)
{
	t_ = t;
	x_ = x;
	u_ = u;
	update();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintsCount()
{
	return getIntermediateConstraintsCount() + getTerminalConstraintsCount();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBoundsIntermediate() const
{
	return lowerBoundsIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBoundsTerminal() const
{
	return lowerBoundsTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBoundsIntermediate() const
{
	return upperBoundsIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerBase<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBoundsTerminal() const
{
	return upperBoundsTerminal_;
}


}  // namespace optcon
}  // namespace ct
