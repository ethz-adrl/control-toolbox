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

#include <gtest/gtest.h>

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
		pDot = (A_ * x + B_ * control).topRows<3>();
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


class TestOscillator : public SymplecticSystem<1, 1, 1>
{
public:

	virtual TestOscillator* clone() const override{
		return new TestOscillator(*this);
	}

	virtual void computePdot(
			const StateVector<2>& x,
			const StateVector<1>& v,
			const ControlVector<1>& control,
			StateVector<1>& pDot
		) override
	{
		pDot = v;
	}

	virtual void computeVdot(
			const StateVector<2>& x,
			const StateVector<1>& p,
			const ControlVector<1>& control,
			StateVector<1>& vDot
		) override
	{
		vDot(0) = control(0) - 10*p(0); // mass is 1 kg
	}
};


//! Linear system class for the GNMS unit test
class LinearizedOscillator : public LinearSystem<2, 1>
{
public:
	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const StateVector<2>& x, const ControlVector<1>& u, const double t = 0.0) override
	{
		A_ << 0, 1, -10, 0;
		return A_;
	}

	const state_control_matrix_t& getDerivativeControl(const StateVector<2>& x, const ControlVector<1>& u, const double t = 0.0) override {
		B_ << 0, 1;
		return B_;
	}

	LinearizedOscillator* clone() const override {
		return new LinearizedOscillator();
	};
};



TEST(LinearSystemDiscretizerTest, testExplicit)
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

		// assert equality between propagated state and the one from the discretizer
		ASSERT_LT((state - stateDiscretizer).array().abs().maxCoeff(), 1e-10);

//		std::cout << "state (reference): "<<state.transpose() << std::endl;
//		std::cout << "stateDiscretizer: "<<stateDiscretizer.transpose() << std::endl;
//		std::cout << "stateWrongDiscretizer: "<<stateWrongDiscretizer.transpose() << std::endl;
	}
}



TEST(LinearSystemDiscretizerTest, testSymplectic)
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
		// assert equality between propagated state and the one from the discretizer
		ASSERT_LT((state - stateDiscretizer).array().abs().maxCoeff(), 1e-10);

//		std::cout << "state (symplectic): "<<state.transpose() << std::endl;
//		std::cout << "state (explicit): "<<stateExplicit.transpose() << std::endl;
//		std::cout << "stateDiscretizer: "<<stateDiscretizer.transpose() << std::endl;
//		std::cout << "stateWrongDiscretizer: "<<stateWrongDiscretizer.transpose() << std::endl;
	}
}



TEST(LinearSystemDiscretizerTest, testExplicitOsc)
{
	double dt = 0.1;

	std::shared_ptr<LinearSystem<2,1>> linearSystem(new LinearizedOscillator());

	std::shared_ptr<ConstantController<2,1>> controller(new ConstantController<2,1>);

	linearSystem->setController(controller);

	Integrator<2> integrator(linearSystem, IntegrationType::EULER);

	LinearSystemDiscretizer<2,1> discretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::FORWARD_EULER);
	LinearSystemDiscretizer<2,1> wrongDiscretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::BACKWARD_EULER);

	size_t nTests = 10;
	size_t numSteps = 100;
	for (size_t i=0; i<nTests; i++)
	{
		StateVector<2> state; state.setRandom();
		StateVector<2> stateDiscretizer; stateDiscretizer = state;
		StateVector<2> stateWrongDiscretizer; stateWrongDiscretizer = state;

		ControlVector<1> control; control.setConstant(1.0);
		controller->setControl(control);

		integrator.integrate_n_steps(state, 0, numSteps, dt);

		for (size_t j=0; j<numSteps; j++)
		{
			Eigen::Matrix<double, 2,2> Ad;
			Eigen::Matrix<double, 2,1> Bd;
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
			Eigen::Matrix<double, 2,2> Ad;
			Eigen::Matrix<double, 2,1> Bd;
			wrongDiscretizer.getAandB(
					stateWrongDiscretizer,
					control,
					0,
					Ad,
					Bd);

			stateWrongDiscretizer = Ad*stateWrongDiscretizer + Bd*control;
		}

		// assert equality between propagated state and the one from the discretizer
		ASSERT_LT((state - stateDiscretizer).array().abs().maxCoeff(), 1e-10);

//		std::cout << "state (reference): "<<state.transpose() << std::endl;
//		std::cout << "stateDiscretizer: "<<stateDiscretizer.transpose() << std::endl;
//		std::cout << "stateWrongDiscretizer: "<<stateWrongDiscretizer.transpose() << std::endl;
	}
}



TEST(LinearSystemDiscretizerTest, testSymplecticOsc)
{
	double dt = 0.1;

	std::shared_ptr<ConstantController<2,1>> controller(new ConstantController<2,1>);

	std::shared_ptr<LinearSystem<2,1>> linearSystem(new LinearizedOscillator());
	linearSystem->setController(controller);
	std::shared_ptr<TestOscillator> linearSymplecticSystem(new TestOscillator());
	linearSymplecticSystem->setController(controller);

	IntegratorSymplecticEuler<1,1,1> integrator(linearSymplecticSystem);
	Integrator<2> integratorExplicit(linearSymplecticSystem, IntegrationType::EULER);

	LinearSystemDiscretizer<2,1> discretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::SYMPLECTIC_EULER);
	LinearSystemDiscretizer<2,1> wrongDiscretizer(dt, linearSystem, LinearSystemDiscretizerSettings::APPROXIMATION::FORWARD_EULER);

	size_t nTests = 10;
	size_t numSteps = 10;
	for (size_t i=0; i<nTests; i++)
	{
		StateVector<2> state; state.setRandom();
		StateVector<2> stateExplicit; stateExplicit = state;
		StateVector<2> stateDiscretizer; stateDiscretizer = state;
		StateVector<2> stateWrongDiscretizer; stateWrongDiscretizer = state;


		ControlVector<1> control; control.setConstant(1.0);
		controller->setControl(control);

		integrator.integrate_n_steps(state, 0, numSteps, dt);
		integratorExplicit.integrate_n_steps(stateExplicit, 0, numSteps, dt);

		for (size_t j=0; j<numSteps; j++)
		{
			Eigen::Matrix<double, 2,2> Ad;
			Eigen::Matrix<double, 2,1> Bd;
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
			Eigen::Matrix<double, 2,2> Ad;
			Eigen::Matrix<double, 2,1> Bd;
			wrongDiscretizer.getAandB(
					stateWrongDiscretizer,
					control,
					0,
					Ad,
					Bd);

			stateWrongDiscretizer = Ad*stateWrongDiscretizer + Bd*control;
		}

		// assert equality between propagated state and the one from the discretizer
		ASSERT_LT((state - stateDiscretizer).array().abs().maxCoeff(), 1e-10);

//		std::cout << "state (explicit): "<<stateExplicit.transpose() << std::endl;
//		std::cout << "stateWrongDiscretizer: "<<stateWrongDiscretizer.transpose() << std::endl;
	}
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
