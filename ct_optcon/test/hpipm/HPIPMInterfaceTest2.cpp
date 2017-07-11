/*
 * HPIPMInterfaceTest2.cpp
 *
 *  Created on: Jul 7, 2017
 *      Author: neunertm
 */

#define HPIPM

#include <ct/optcon/optcon.h>

using namespace ct::core;
using namespace ct::optcon;

const size_t state_dim = 2; // position, velocity
const size_t control_dim = 1; // force

const double kStiffness = 10;

//! Dynamics class for the GNMS unit test
class Dynamics : public ControlledSystem<state_dim, control_dim>
{
public:
	Dynamics() : ControlledSystem<state_dim, control_dim>(SYSTEM_TYPE::SECOND_ORDER) {}

	void computeControlledDynamics(
			const StateVector<state_dim>& state,
			const Time& t,
			const ControlVector<control_dim>& control,
			StateVector<state_dim>& derivative
	) override
	{
		derivative(0) = state(1);
		derivative(1) = control(0) - kStiffness*state(0) + 0.1; // mass is 1 kg
	}

	Dynamics* clone() const override
	{
		return new Dynamics();
	};
};

//! Linear system class for the GNMS unit test
class LinearizedSystem : public LinearSystem<state_dim, control_dim>
{
public:
	state_matrix_t A_;
	state_control_matrix_t B_;


	const state_matrix_t& getDerivativeState(const StateVector<state_dim>& x, const ControlVector<control_dim>& u, const double t = 0.0) override
			{
		A_ << 0, 1, -kStiffness, 0;
		return A_;
			}

	const state_control_matrix_t& getDerivativeControl(const StateVector<state_dim>& x, const ControlVector<control_dim>& u, const double t = 0.0) override
			{

		B_ << 0, 1;
		return B_;
			}

	LinearizedSystem* clone() const override
			{
		return new LinearizedSystem();
			};
};


//! Create a cost function for the GNMS unit test
std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > createCostFunction(Eigen::Vector2d& x_final)
{
	Eigen::Matrix2d Q;
	Q << 0, 0, 0, 0;

	Eigen::Matrix<double, 1, 1> R;
	R << 1.0;

	Eigen::Vector2d x_nominal = Eigen::Vector2d::Zero();
	Eigen::Matrix<double, 1, 1> u_nominal = Eigen::Matrix<double, 1, 1>::Zero();

	Eigen::Matrix2d Q_final;
	Q_final << 1000, 0, 0, 10;

	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > quadraticCostFunction(
			new CostFunctionQuadraticSimple<state_dim, control_dim>(
					Q, R, x_nominal, u_nominal, x_final, Q_final));

	return quadraticCostFunction;
}

void testGNMS();

int main(int argc, char* argv[])
{

	LinearizedSystem system;

	int N = 5;

	double dt = 0.1;

	ct::optcon::HPIPMInterface<state_dim, control_dim> interface;
	StateVector<state_dim> x0;
	x0 << 1.0, 0.0;


	StateMatrix<state_dim> Q;
	Q.setZero();
	ControlMatrix<control_dim> R;
	R.setIdentity();

	StateVector<state_dim> xNom;
	xNom.setZero();

	ControlVector<control_dim> uNom;
	uNom.setZero();

	StateMatrix<state_dim> Qfinal;
	Qfinal << 1000, 0, 0, 10;

	ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim> costFunction(Q, R,
			xNom, uNom,
			xNom, Qfinal);

	interface.changeTimeHorizon(N);
	interface.solveLinearProblem(x0, system, costFunction, xNom, dt);
	interface.printSolution();

	std::cout <<std::endl <<std::endl << std::endl;
	std::cout << "TEST GNMS!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl <<std::endl << std::endl;

	testGNMS();

	return 1;
}



void testGNMS()
{

	std::cout << "setting up problem " << std::endl;

	Eigen::Vector2d x_final;
	x_final << 20, 0;

	GNMSSettings gnms_settings;
	gnms_settings.epsilon = 0.0;
	gnms_settings.nThreads = 4;
	gnms_settings.max_iterations = 3;
	gnms_settings.recordSmallestEigenvalue = true;
	gnms_settings.min_cost_improvement = 1e-6;
	gnms_settings.fixedHessianCorrection = false;
	gnms_settings.dt = 0.1;
	gnms_settings.dt_sim = 0.1;
	gnms_settings.integrator = GNMSSettings::EULER;
	gnms_settings.discretization = GNMSSettings::DISCRETIZATION::FORWARD_EULER;


	std::shared_ptr<ControlledSystem<state_dim, control_dim> > nonlinearSystem(new Dynamics);
	std::shared_ptr<LinearSystem<state_dim, control_dim> > analyticLinearSystem(new LinearizedSystem);
	std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim> > costFunction = createCostFunction(x_final);

	// times
	ct::core::Time tf = 5*gnms_settings.dt;
	size_t nSteps = 5;

	// provide initial guess
	ControlVectorArray<control_dim> u0(nSteps, ControlVector<control_dim>::Zero());
	StateVectorArray<state_dim>  x0(nSteps+1, StateVector<state_dim>::Zero());
	x0[0] << 1.0, 0.0;

	GNMS<state_dim, control_dim>::Policy_t initController (u0, x0);

	// construct single-core single subsystem OptCon Problem
	OptConProblem<state_dim, control_dim> optConProblem (tf, x0[0], nonlinearSystem, costFunction, analyticLinearSystem);


	std::cout << "initializing gnms solver" << std::endl;
	GNMS<state_dim, control_dim> gnms(optConProblem, gnms_settings);

	gnms.configure(gnms_settings);
	gnms.setInitialGuess(initController);



	std::cout << "running gnms solver" << std::endl;

	bool foundBetter = true;
	foundBetter = gnms.runIteration();

	// test trajectories
	StateTrajectory<state_dim> xRollout = gnms.getStateTrajectory();
	ControlTrajectory<control_dim> uRollout = gnms.getControlTrajectory();

	std::cout<<"x final GNMS: " << std::endl;
	for (size_t i=0; i<xRollout.size(); i++)
		std::cout<<xRollout[i].transpose() << std::endl;

	std::cout<<"u final GNMS: " << std::endl;
	for (size_t i=0; i<uRollout.size(); i++)
		std::cout<<uRollout[i].transpose() << std::endl;

}
