/*
 * LQOCSolverTest.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: neunertm
 */

#include <ct/core/core.h>

using namespace ct;
using namespace ct::optcon;

#include "../../testSystems/DiehlSystem.h"

int main(int argc, char* argv[])
{
	const size_t state_dim = 1;
	const size_t control_dim = 1;
	const size_t N = 5;
	const double dt = 0.5;

	std::vector<std::shared_ptr<LQOCSolver>> lqocSolvers;

	std::shared_ptr<LQOCSolver> hpipmSolver(new HPIPMInterface);
	std::shared_ptr<LQOCSolver> gnRiccatiSolver(new GNRiccatiSolver);
	lqocSolvers.push_back(hpipmSolver);
	lqocSolvers.push_back(gnRiccatiSolver);

	LQOCProblem<state_dim, control_dim> lqocProblem(5);
	std::shared_ptr<core::LinearSystem<state_dim, control_dim>> springLoadedMassLinear(new SpringLoadedMassLinear());
	core::LinearSystemDiscretizer<state_dim, control_dim> discreteSpringLoadedMass(springLoadedMassLinear, dt);

	ct::core::StateVector<state_dim> x0;
	x0 << 0.05;

	auto costFunction = createDiehlCostFunction(x0);

	ct::core::StateVector<state_dim> b;
	b << 0.1; // for DiehlSystem

	lqocProblem.setFromLinearQuadraticProblem(
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

		for (size_t j=0; j<xSol.size(); j++)
			std::cout<<xSol[j].transpose()<<std::endl;

		for (size_t j=0; j<uSol.size(); j++)
			std::cout<<uSol[j].transpose()<<std::endl;
	}

	return 1;
}
