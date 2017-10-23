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


#include <Eigen/Dense>
#include <ct/core/core.h>
#include "Quadrotor.hpp"


namespace ct {
namespace models {

class QuadrotorLinear : public ct::core::LinearSystem<quadrotor::nStates, quadrotor::nControls>
{
public:
	typedef ct::core::StateVector<quadrotor::nStates> state_vector_t;
	typedef ct::core::ControlVector<quadrotor::nControls> control_vector_t;

	typedef Eigen::Matrix<double, quadrotor::nStates, quadrotor::nStates> state_matrix_t;
	typedef Eigen::Matrix<double, quadrotor::nStates, quadrotor::nControls> state_control_matrix_t;


	virtual QuadrotorLinear* clone() const override { return new QuadrotorLinear(*this); }
	virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
		const control_vector_t& u,
		const ct::core::Time t = 0.0) override
	{
		A_ = A_quadrotor(x, u);
		return A_;
	}

	virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
		const control_vector_t& u,
		const ct::core::Time t = 0.0) override
	{
		B_ = B_quadrotor(x, u);
		return B_;
	}

private:
	state_matrix_t A_;
	state_control_matrix_t B_;
};
}
}
