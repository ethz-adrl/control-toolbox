/*
 * DiehlProblem.h
 *
 *  Created on: Jul 13, 2017
 *      Author: neunertm
 */

#ifndef TEST_TESTSYSTEMS_DIEHLSYSTEM_H_
#define TEST_TESTSYSTEMS_DIEHLSYSTEM_H_

namespace ct {
namespace optcon {
namespace example {


//! Dynamics class for the GNMS unit test
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

//! Linear system class for the GNMS unit test
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


std::shared_ptr<core::CostFunctionQuadratic<1, 1> > createDiehlCostFunction(const core::StateVector<1>& x_final)
{
	Eigen::Matrix<double, 1,1 > Q;
	Q << 1.0;

	Eigen::Matrix<double, 1, 1> R;
	R << 1.0;

	Eigen::Matrix<double, 1, 1> x_nominal = Eigen::Matrix<double, 1, 1>::Zero();
	Eigen::Matrix<double, 1, 1> u_nominal = Eigen::Matrix<double, 1, 1>::Zero();

	Eigen::Matrix<double, 1, 1> Q_final;
	Q_final << 10.0;

	std::shared_ptr<core::CostFunctionQuadratic<1, 1> > quadraticCostFunction(
			new core::CostFunctionQuadraticSimple<1, 1>(
					Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}


}
}
}



#endif /* TEST_TESTSYSTEMS_DIEHLSYSTEM_H_ */
