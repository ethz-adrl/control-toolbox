/*
 * LQOCSolverTest.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: neunertm
 */

#include <ct/optcon/optcon.h>

using namespace ct;
using namespace ct::optcon;

#include "../../testSystems/MIMOIntegrator.h"

const double dt = 0.1;

template <size_t control_dim, size_t state_dim>
void timeSingleSolve(size_t N)
{
	std::cout << "time horizon: "<<N<<std::endl;
	std::cout << "============= " << std::endl << std::endl;

	std::vector<std::shared_ptr<LQOCSolver<state_dim, control_dim>>> lqocSolvers;

	std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>);
	std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>);
	std::vector<std::string> solverNames = {"Riccati",  "HPIPM" };

	lqocSolvers.push_back(gnRiccatiSolver);
	lqocSolvers.push_back(hpipmSolver);

	std::vector<std::shared_ptr<LQOCProblem<state_dim, control_dim>>> problems;
	std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));
	std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem2(new LQOCProblem<state_dim, control_dim>(N));

	problems.push_back(lqocProblem1);
	problems.push_back(lqocProblem2);

	std::shared_ptr<core::LinearSystem<state_dim, control_dim>> exampleSystem(new example::MIMOIntegratorLinear<state_dim, control_dim>());
	core::LinearSystemDiscretizer<state_dim, control_dim> discreteExampleSystem(dt, exampleSystem, core::LinearSystemDiscretizerSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

	ct::core::ControlVector<control_dim> u0; u0.setRandom();
	ct::core::StateVector<state_dim> x0; x0.setRandom();
	ct::core::StateVector<state_dim> xf; xf.setRandom();

	auto costFunction = example::createMIMOIntegratorCostFunction<state_dim,control_dim>(xf);

	ct::core::StateVector<state_dim> b; b.setZero();


	for (size_t i=0; i<lqocSolvers.size(); i++)
	{
			problems[i]->setFromTimeInvariantLinearQuadraticProblem(
					x0,
					u0,
					discreteExampleSystem,
					*costFunction,
					b,
					dt
			);

			auto start_all = std::chrono::steady_clock::now();

			auto start_set = std::chrono::steady_clock::now();
			lqocSolvers[i]->setProblem(problems[i]);
			auto end_set = std::chrono::steady_clock::now();

			auto start_solve = std::chrono::steady_clock::now();
			lqocSolvers[i]->solve();
			auto end_solve = std::chrono::steady_clock::now();

			auto start_get= std::chrono::steady_clock::now();
			auto xSol = lqocSolvers[i]->getSolutionState();
			auto uSol = lqocSolvers[i]->getSolutionControl();
			auto end_get = std::chrono::steady_clock::now();

			auto end_all = std::chrono::steady_clock::now();

			std::cout << "setProblem() with "<<solverNames[i] << " took " <<std::chrono::duration <double, std::milli> (end_set-start_set).count() << " ms" <<std::endl;
			std::cout << "solve() with "<<solverNames[i] << " took " <<std::chrono::duration <double, std::milli> (end_solve-start_solve).count() << " ms" <<std::endl;
			std::cout << "get() with "<<solverNames[i] << " took " <<std::chrono::duration <double, std::milli> (end_get-start_get).count() << " ms" <<std::endl;
			std::cout << "total call with "<<solverNames[i] << " took " <<std::chrono::duration <double, std::milli> (end_all-start_all).count() << " ms" <<std::endl;

//			std::cout << "x:" << std::endl;
//			for (size_t j=0; j<xSol.size(); j++)
//				std::cout<<xSol[j].transpose()<<std::endl;
//
//			std::cout << "u:" << std::endl;
//			for (size_t j=0; j<uSol.size(); j++)
//				std::cout<<uSol[j].transpose()<<std::endl;

			std::cout << std::endl;

	}
	std::cout << std::endl;
}


template <size_t control_dim, size_t state_dim>
void timeSolvers()
{
	std::vector<size_t> testTimeHorizons = { 5, 50, 500, 5000, 10000 };

	std::cout << "Test System: "<<std::endl;
	std::cout << "============ "<<std::endl;
	std::cout << "state dim: "<<state_dim<<std::endl;
	std::cout << "control dim: "<<control_dim<<std::endl;


	for (size_t i=0; i<testTimeHorizons.size(); i++)
	{
		timeSingleSolve<control_dim, state_dim>(testTimeHorizons[i]);
	}

	std::cout << std::endl << std::endl << std::endl << std::endl;
}


int main(int argc, char* argv[])
{
	timeSolvers<3,3>();
	timeSolvers<6,6>();
	timeSolvers<12,12>();
	timeSolvers<24,24>();
	timeSolvers<48,48>();

	return 1;
}
