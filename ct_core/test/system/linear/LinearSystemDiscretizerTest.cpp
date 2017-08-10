/*
 * LinearSystemDiscretizerTest.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: neunertm
 */

#include <ct/core/core.h>

using namespace ct::core;


Eigen::Matrix<double, 6, 6> A()
{
	Eigen::Matrix<double, 6, 6> A;
	A.setIdentity();
	A.template topRightCorner<3,3>().setIdentity();
	A.template bottomLeftCorner<3,3>().setIdentity();
	A = -0.1*A;
	return A;
}
Eigen::Matrix<double, 6, 3> B()
{
	Eigen::Matrix<double, 6, 3> B;
	B.setZero();
	B.template topRows<3>().setIdentity();
	B.template bottomRows<3>().setIdentity();
	return B;
}



class TestLinearSymplecticSystem : public SymplecticSystem<3, 3, 3>
{
public:
	TestLinearSymplecticSystem()
	{
		A_ = A();
		B_ = B();
	}

	virtual TestLinearSymplecticSystem* clone() const override{
		return new TestLinearSymplecticSystem(*this);
	}

	virtual void computePdot(
			const StateVector<3 + 3>& x,
			const StateVector<3>& v,
			const ControlVector<3>& control,
			StateVector<3>& pDot
		) override
	{
		StateVector<3 + 3> xlocal;
		xlocal = x;
		xlocal.template bottomRows<3>() = v;

		pDot = (A_ * xlocal + B_ * control).topRows<3>();
	}

	virtual void computeVdot(
			const StateVector<3 + 3>& x,
			const StateVector<3>& p,
			const ControlVector<3>& control,
			StateVector<3>& vDot
		) override
	{
		vDot = (A_ * x + B_ * control).bottomRows<3>();
	}

private:
	Eigen::Matrix<double, 6, 6> A_;
	Eigen::Matrix<double, 6, 3> B_;
};

void testExplicit()
{
	double dt = 0.1;

	std::shared_ptr<LinearSystem<6,3>> linearSystem(new LTISystem<6, 3>(A(),B()));

	std::shared_ptr<ConstantController<6,3>> controller(new ConstantController<6,3>);

	linearSystem->setController(controller);

	Integrator<6> integrator(linearSystem, IntegrationType::EULER);

	LinearSystemDiscretizer<6,3> discretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::FORWARD_EULER);
	LinearSystemDiscretizer<6,3> wrongDiscretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::BACKWARD_EULER);

	size_t nTests = 10;
	size_t numSteps = 100;
	for (size_t i=0; i<nTests; i++)
	{
		StateVector<6> state; state.setRandom();
		StateVector<6> stateDiscretizer; stateDiscretizer = state;
		StateVector<6> stateWrongDiscretizer; stateWrongDiscretizer = state;

		ControlVector<3> control; control.setConstant(1.0);
		controller->setControl(control);

		integrator.integrate_n_steps(state, 0, numSteps, dt);

		for (size_t j=0; j<numSteps; j++)
		{
			Eigen::Matrix<double, 6, 6> Ad;
			Eigen::Matrix<double, 6, 3> Bd;
			discretizer.getAandB(
					stateDiscretizer,
					control,
					0,
					Ad,
					Bd);

			stateDiscretizer = Ad*stateDiscretizer + Bd*control;
		}

		for (size_t j=0; j<numSteps; j++)
		{
			Eigen::Matrix<double, 6, 6> Ad;
			Eigen::Matrix<double, 6, 3> Bd;
			wrongDiscretizer.getAandB(
					stateWrongDiscretizer,
					control,
					0,
					Ad,
					Bd);

			stateWrongDiscretizer = Ad*stateWrongDiscretizer + Bd*control;
		}
		std::cout << "state (reference): "<<state.transpose() << std::endl;
		std::cout << "stateDiscretizer: "<<stateDiscretizer.transpose() << std::endl;
		std::cout << "stateWrongDiscretizer: "<<stateWrongDiscretizer.transpose() << std::endl;
	}
}



void testSymplectic()
{
	double dt = 0.1;

	std::shared_ptr<ConstantController<6,3>> controller(new ConstantController<6,3>);

	std::shared_ptr<LinearSystem<6,3>> linearSystem(new LTISystem<6, 3>(A(),B()));
	linearSystem->setController(controller);
	std::shared_ptr<TestLinearSymplecticSystem> linearSymplecticSystem(new TestLinearSymplecticSystem);
	linearSymplecticSystem->setController(controller);

	IntegratorSymplecticEuler<3, 3, 3> integrator(linearSymplecticSystem);
	Integrator<6> integratorExplicit(linearSymplecticSystem, IntegrationType::EULER);

	LinearSystemDiscretizer<6,3> discretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::SYMPLECTIC_EULER);
	LinearSystemDiscretizer<6,3> wrongDiscretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::FORWARD_EULER);

	size_t nTests = 10;
	size_t numSteps = 10;
	for (size_t i=0; i<nTests; i++)
	{
		StateVector<6> state; state.setRandom();
		StateVector<6> stateExplicit; stateExplicit = state;
		StateVector<6> stateDiscretizer; stateDiscretizer = state;
		StateVector<6> stateWrongDiscretizer; stateWrongDiscretizer = state;


		ControlVector<3> control; control.setConstant(1.0);
		controller->setControl(control);

		integrator.integrate_n_steps(state, 0, numSteps, dt);
		integratorExplicit.integrate_n_steps(stateExplicit, 0, numSteps, dt);

		for (size_t j=0; j<numSteps; j++)
		{
			Eigen::Matrix<double, 6, 6> Ad;
			Eigen::Matrix<double, 6, 3> Bd;
			discretizer.getAandB(
					stateDiscretizer,
					control,
					0,
					Ad,
					Bd);

			stateDiscretizer = Ad*stateDiscretizer + Bd*control;
		}

		for (size_t j=0; j<numSteps; j++)
		{
			Eigen::Matrix<double, 6, 6> Ad;
			Eigen::Matrix<double, 6, 3> Bd;
			wrongDiscretizer.getAandB(
					stateWrongDiscretizer,
					control,
					0,
					Ad,
					Bd);

			stateWrongDiscretizer = Ad*stateWrongDiscretizer + Bd*control;
		}
		std::cout << "state (symplectic): "<<state.transpose() << std::endl;
		std::cout << "state (explicit): "<<stateExplicit.transpose() << std::endl;
		std::cout << "stateDiscretizer: "<<stateDiscretizer.transpose() << std::endl;
		std::cout << "stateWrongDiscretizer: "<<stateWrongDiscretizer.transpose() << std::endl;
	}
}

int main(int argc, char** argv)
{
	std::cout << "Testing explicit"<<std::endl << std::endl;
	testExplicit();
	std::cout << "Finished testing explicit"<<std::endl << std::endl;

	std::cout << "Testing symplectic"<<std::endl << std::endl;
	testSymplectic();
	std::cout << "Finished testing symplectic"<<std::endl << std::endl;

	return 1;
}
