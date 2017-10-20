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


//! Dynamics class for the GNMS unit test
class SpringLoadedMass : public core::ControlledSystem<2, 1>
{
public:
	static const size_t state_dim = 2;    // position, velocity
	static const size_t control_dim = 1;  // force

	SpringLoadedMass() : core::ControlledSystem<state_dim, control_dim>(core::SYSTEM_TYPE::SECOND_ORDER) {}
	void computeControlledDynamics(const core::StateVector<state_dim>& state,
		const core::Time& t,
		const core::ControlVector<control_dim>& control,
		core::StateVector<state_dim>& derivative) override
	{
		derivative(0) = state(1);
		derivative(1) = control(0) - kStiffness * state(0) + 0.1;  // mass is 1 kg
	}

	SpringLoadedMass* clone() const override { return new SpringLoadedMass(); };
	static constexpr double kStiffness = 10;
};

//! Linear system class for the GNMS unit test
class SpringLoadedMassLinear : public core::LinearSystem<2, 1>
{
public:
	static const size_t state_dim = 2;    // position, velocity
	static const size_t control_dim = 1;  // force

	static constexpr double kStiffness = 10;

	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const core::StateVector<state_dim>& x,
		const core::ControlVector<control_dim>& u,
		const double t = 0.0) override
	{
		A_ << 0, 1, -kStiffness, 0;
		return A_;
	}

	const state_control_matrix_t& getDerivativeControl(const core::StateVector<state_dim>& x,
		const core::ControlVector<control_dim>& u,
		const double t = 0.0) override
	{
		B_ << 0, 1;
		return B_;
	}

	SpringLoadedMassLinear* clone() const override { return new SpringLoadedMassLinear(); };
};


std::shared_ptr<CostFunctionQuadratic<2, 1>> createSpringLoadedMassCostFunction(const core::StateVector<2>& x_final)
{
	Eigen::Matrix<double, 2, 2> Q;
	Q << 1.0, 0, 0, 1.0;

	Eigen::Matrix<double, 1, 1> R;
	R << 1.0;

	Eigen::Matrix<double, 2, 1> x_nominal = x_final;
	Eigen::Matrix<double, 1, 1> u_nominal;
	u_nominal.setZero();

	Eigen::Matrix<double, 2, 2> Q_final;
	Q_final << 10.0, 0, 0, 10.0;

	std::shared_ptr<CostFunctionQuadratic<2, 1>> quadraticCostFunction(
		new CostFunctionQuadraticSimple<2, 1>(Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}
}
}
}
