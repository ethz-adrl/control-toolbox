/*
 * SpringLoadedMass.h
 *
 *  Created on: Jul 13, 2017
 *      Author: neunertm
 */

#ifndef TEST_TESTSYSTEMS_SPRINGLOADEDMASS_H_
#define TEST_TESTSYSTEMS_SPRINGLOADEDMASS_H_


namespace ct {
namespace optcon {
namespace example {


//! Dynamics class for the GNMS unit test
class SpringLoadedMass : public core::ControlledSystem<2, 1>
{
public:
	static const size_t state_dim = 2; // position, velocity
	static const size_t control_dim = 1; // force

	SpringLoadedMass() : core::ControlledSystem<state_dim, control_dim>(core::SYSTEM_TYPE::SECOND_ORDER) {}

	void computeControlledDynamics(
			const core::StateVector<state_dim>& state,
			const core::Time& t,
			const core::ControlVector<control_dim>& control,
			core::StateVector<state_dim>& derivative
	) override
	{
		derivative(0) = state(1);
		derivative(1) = control(0) - kStiffness*state(0) + 0.1; // mass is 1 kg
	}

	SpringLoadedMass* clone() const override
	{
		return new SpringLoadedMass();
	};

	static constexpr double kStiffness = 10;
};

//! Linear system class for the GNMS unit test
class SpringLoadedMassLinear : public core::LinearSystem<2, 1>
{
public:
	static const size_t state_dim = 2; // position, velocity
	static const size_t control_dim = 1; // force

	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const core::StateVector<state_dim>& x, const core::ControlVector<control_dim>& u, const double t = 0.0) override
	{
		A_ << 0, 1, -kStiffness, 0;
		return A_;
	}

	const state_control_matrix_t& getDerivativeControl(const core::StateVector<state_dim>& x, const core::ControlVector<control_dim>& u, const double t = 0.0) override
	{
		B_ << 0, 1;
		return B_;
	}

	SpringLoadedMassLinear* clone() const override
	{
		return new SpringLoadedMassLinear();
	};
};

}
}
}


#endif /* TEST_TESTSYSTEMS_SPRINGLOADEDMASS_H_ */
