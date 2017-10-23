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
CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunction() : t_(0.0), t_shift_(0.0)
{
    x_.setZero();
    u_.setZero();
};

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::~CostFunction(){};

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunction(const CostFunction& arg)
    : x_(arg.x_), u_(arg.u_), t_(arg.t_), t_shift_(arg.t_shift_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    x_ = x;
    u_ = u;
    t_ = t + t_shift_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::getCurrentStateAndControl(Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    SCALAR& t) const
{
    x = this->x_;
    u = this->u_;
    t = this->t_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunction<STATE_DIM, CONTROL_DIM, SCALAR>::shiftTime(const SCALAR t)
{
    t_shift_ = t;
}

}  // namespace optcon
}  // namespace ct
