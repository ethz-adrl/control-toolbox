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
namespace example {


/*!
 *
 * We named this system after Prof. Moritz Diehl from the University of Freiburg, who gave it to us as a simple 1-dimensional test system.
 * The system dynamics are dx/dt = (1+x)x + u + 0.1
 *
 */
//! Dynamics class for the Diehl system
class DiehlSystem : public core::ControlledSystem<1, 1>
{
public:
	static const int state_dim = 1;
	static const int control_dim = 1;

	DiehlSystem() : core::ControlledSystem<1, 1>(core::SYSTEM_TYPE::SECOND_ORDER) {}

	void computeControlledDynamics(
			const core::StateVector<1>& state,
			const core::Time& t,
			const core::ControlVector<1>& control,
			core::StateVector<1>& derivative
	) override
	{
		derivative(0) = (1.0 + state(0)) * state(0) + control(0) + 0.1;
	}

	DiehlSystem* clone() const override
	{
		return new DiehlSystem();
	};
};

//! Linear system class for the Diehl system
class DiehlSystemLinear : public core::LinearSystem<1, 1>
{
public:
	static const int state_dim = 1;
	static const int control_dim = 1;

	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const core::StateVector<1>& x, const core::ControlVector<1>& u, const double t = 0.0) override
	{
		A_ << 1+2*x(0);
		return A_;
	}

	const state_control_matrix_t& getDerivativeControl(const core::StateVector<1>& x, const core::ControlVector<1>& u, const double t = 0.0) override
	{
		B_ << 1;
		return B_;
	}

	DiehlSystemLinear* clone() const override
	{
		return new DiehlSystemLinear();
	}
};


//! create a cost function of appropriate Dimensions for the Diehl system
std::shared_ptr<CostFunctionQuadratic<1, 1> > createDiehlCostFunction(const core::StateVector<1>& x_final)
{
	Eigen::Matrix<double, 1,1 > Q;
	Q << 1.0;

	Eigen::Matrix<double, 1, 1> R;
	R << 1.0;

	Eigen::Matrix<double, 1, 1> x_nominal = x_final;
	Eigen::Matrix<double, 1, 1> u_nominal; u_nominal.setConstant(-0.1); //Eigen::Matrix<double, 1, 1>::Zero();

	Eigen::Matrix<double, 1, 1> Q_final;
	Q_final << 10.0;

	std::shared_ptr<CostFunctionQuadratic<1, 1> > quadraticCostFunction(
			new CostFunctionQuadraticSimple<1, 1>(
					Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}


}
}
}
