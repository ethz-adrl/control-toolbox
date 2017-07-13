/*
 * LQOCSolverTest.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: neunertm
 */

#include <ct/optcon/optcon.h>

using namespace ct;
using namespace ct::optcon;

#include "../../testSystems/DiehlSystem.h"

int main(int argc, char* argv[])
{
	const size_t state_dim = 1;
	const size_t control_dim = 1;
	const size_t N = 5;
	const double dt = 0.5;

	std::vector<std::shared_ptr<LQOCSolver<state_dim, control_dim>>> lqocSolvers;

	std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);
	std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>);
	std::vector<std::string> solverNames = { "HPIPM", "Riccati" };

	lqocSolvers.push_back(hpipmSolver);
	lqocSolvers.push_back(gnRiccatiSolver);

	std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem(new LQOCProblem<state_dim, control_dim>(5));
	std::shared_ptr<core::LinearSystem<state_dim, control_dim>> springLoadedMassLinear(new example::DiehlSystemLinear());
	core::LinearSystemDiscretizer<state_dim, control_dim> discreteSpringLoadedMass(springLoadedMassLinear, dt);

	ct::core::StateVector<state_dim> x0;
	x0 << 0.05;

	auto costFunction = example::createDiehlCostFunction(x0);

	ct::core::StateVector<state_dim> b;
	b << 0.1; // for DiehlSystem

	lqocProblem->setFromTimeInvariantLinearQuadraticProblem(
		x0,
		discreteSpringLoadedMass,
		*costFunction,
		b,
		dt
	);

	for (size_t i=0; i<lqocSolvers.size(); i++)
	{
		lqocSolvers[i]->setProblem(lqocProblem);
		lqocSolvers[i]->solve();
		auto xSol = lqocSolvers[i]->getSolutionState();
		auto uSol = lqocSolvers[i]->getSolutionControl();

		std::cout << "Solution for "<<solverNames[i]<<std::endl;

		std::cout << "x:" << std::endl;
		for (size_t j=0; j<xSol.size(); j++)
			std::cout<<xSol[j].transpose()<<std::endl;

		std::cout << "u:" << std::endl;
		for (size_t j=0; j<uSol.size(); j++)
			std::cout<<uSol[j].transpose()<<std::endl;

		std::cout << std::endl << std::endl;
	}

	return 1;
}
