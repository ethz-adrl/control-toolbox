/*
 * HPIPMInterfaceTest3.cpp
 *
 *  Created on: Jul 7, 2017
 *      Author: neunertm
 */

//#define HPIPM

#include <ct/optcon/optcon.h>

using namespace ct::core;
using namespace ct::optcon;


const size_t state_dim = 1;    // position, velocity
const size_t control_dim = 1;  // force

//! Dynamics class for the GNMS unit test
class Dynamics : public ControlledSystem<state_dim, control_dim>
{
public:
	Dynamics() : ControlledSystem<state_dim, control_dim>(SYSTEM_TYPE::SECOND_ORDER) {}
	void computeControlledDynamics(const StateVector<state_dim>& state,
		const Time& t,
		const ControlVector<control_dim>& control,
		StateVector<state_dim>& derivative) override
	{
		derivative(0) = (1.0 + state(0)) * state(0) + control(0) + 0.1;
	}

	Dynamics* clone() const override { return new Dynamics(); };
};

//! Linear system class for the GNMS unit test
class LinearizedSystem : public LinearSystem<state_dim, control_dim>
{
public:
	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x,
		const ControlVector<control_dim>& u,
		const double t = 0.0) override
	{
		A_ << 1 + 2 * x(0);
		return A_;
	}

	const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x,
		const ControlVector<control_dim>& u,
		const double t = 0.0) override
	{
		B_ << 1;
		return B_;
	}

	LinearizedSystem* clone() const override { return new LinearizedSystem(); }
};

void testGNMS();

int main(int argc, char* argv[])
{
	testGNMS();

	return 1;
}


void testGNMS()
{
	std::cout << "setting up problem " << std::endl;

	std::string configFile = "solver.info";
	std::string costFunctionFile = "cost.info";

	Eigen::Matrix<double, 1, 1> x_0;
	ct::core::loadMatrix(costFunctionFile, "x_0", x_0);

	Eigen::Matrix<double, 1, 1> x_f;
	ct::core::loadMatrix(costFunctionFile, "term1.weigths.x_des", x_f);

	GNMSSettings gnms_settings;
	gnms_settings.load(configFile);

	iLQGSettings ilqg_settings;
	ilqg_settings.load(configFile);

	std::shared_ptr<ControlledSystem<state_dim, control_dim>> nonlinearSystem(new Dynamics);
	std::shared_ptr<LinearSystem<state_dim, control_dim>> analyticLinearSystem(new LinearizedSystem);
	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
		new CostFunctionAnalytical<state_dim, control_dim>(costFunctionFile));

	// times
	ct::core::Time tf = 3.0;
	ct::core::loadScalar(configFile, "timeHorizon", tf);

	size_t nSteps = ilqg_settings.computeK(tf);

	// provide initial guess
	ControlVectorArray<control_dim> u0(nSteps, ControlVector<control_dim>::Zero());
	StateVectorArray<state_dim> x0(nSteps + 1, x_0);

	int initType = 0;
	ct::core::loadScalar(configFile, "initType", initType);

	switch (initType)
	{
		case 0:  // zero
			break;

		case 1:  // linear
		{
			for (size_t i = 0; i < nSteps + 1; i++)
			{
				x0[i] = x_0 + (x_f - x_0) * double(i) / double(nSteps);
			}
			break;
		}
		case 2:  // integration
		{
			std::shared_ptr<ControlledSystem<state_dim, control_dim>> systemForInit(new Dynamics);
			ct::core::IntegratorEuler<state_dim> integratorForInit(systemForInit);
			x0[0] = x_0;
			for (size_t i = 1; i < nSteps + 1; i++)
			{
				x0[i] = x0[i - 1];
				integratorForInit.integrate_n_steps(x0[i], 0, 1, ilqg_settings.dt_sim);
			}
			break;
		}
		case 3:  // random
		{
			for (size_t i = 1; i < nSteps + 1; i++)
			{
				x0[i].setRandom();
			}
			break;
		}
		case 4:  // zero
		{
			for (size_t i = 1; i < nSteps + 1; i++)
			{
				x0[i].setZero();
			}
			break;
		}
		default:
		{
			throw std::runtime_error("illegal init type");
			break;
		}
	}

	GNMS<state_dim, control_dim>::Policy_t initController(u0, x0);

	// construct single-core single subsystem OptCon Problem
	OptConProblem<state_dim, control_dim> optConProblem(tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


	std::cout << "initializing gnms solver" << std::endl;
	GNMS<state_dim, control_dim> gnms(optConProblem, gnms_settings);


	gnms.configure(gnms_settings);
	gnms.setInitialGuess(initController);


	std::cout << "running gnms solver" << std::endl;

	bool foundBetter = true;
	size_t numIterations = 0;

	while (foundBetter)
	{
		foundBetter = gnms.runIteration();

		// test trajectories
		StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
		ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

		numIterations++;

		std::cout << "x final GNMS: " << std::endl;
		for (size_t i = 0; i < xRollout.size(); i++)
			std::cout << xRollout[i].transpose() << std::endl;

		std::cout << "u final GNMS: " << std::endl;
		for (size_t i = 0; i < uRollout.size(); i++)
			std::cout << uRollout[i].transpose() << std::endl;

		if (numIterations > 5)
		{
			break;
		}
		int a;
		std::cin >> a;
	}
}
