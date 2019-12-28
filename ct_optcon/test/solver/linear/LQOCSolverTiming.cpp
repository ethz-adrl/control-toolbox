/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * This executable serves to compare the run-times of our own Riccati LQ problem solver and Gianluca Frison's HPIPM solver.
 * It is not a supposed to be a unit test, but can be used to compare runtimes on different machines.
 */

#include <ct/optcon/optcon.h>

using namespace ct;
using namespace ct::optcon;

#include "../../testSystems/MIMOIntegrator.h"

const double dt = 0.1;

template <size_t control_dim, size_t state_dim>
void timeSingleSolve(size_t N, std::vector<std::vector<double>>& loggedSolveTimes)
{
    std::cout << "time horizon: " << N << std::endl;
    std::cout << "============= " << std::endl << std::endl;

    std::vector<std::shared_ptr<LQOCSolver<state_dim, control_dim>>> lqocSolvers;

    std::shared_ptr<LQOCSolver<state_dim, control_dim>> hpipmSolver(new HPIPMInterface<state_dim, control_dim>());
    std::shared_ptr<LQOCSolver<state_dim, control_dim>> gnRiccatiSolver(new GNRiccatiSolver<state_dim, control_dim>());
    std::vector<std::string> solverNames = {"Riccati", "HPIPM"};

    NLOptConSettings solverSettings;
    solverSettings.fixedHessianCorrection = true;
    solverSettings.epsilon = 0;
    solverSettings.recordSmallestEigenvalue = false;
    solverSettings.nThreadsEigen = 1;

    gnRiccatiSolver->configure(solverSettings);
    hpipmSolver->configure(solverSettings);

    lqocSolvers.push_back(gnRiccatiSolver);
    lqocSolvers.push_back(hpipmSolver);

    loggedSolveTimes.resize(lqocSolvers.size());

    std::vector<std::shared_ptr<LQOCProblem<state_dim, control_dim>>> problems;
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem1(new LQOCProblem<state_dim, control_dim>(N));
    std::shared_ptr<LQOCProblem<state_dim, control_dim>> lqocProblem2(new LQOCProblem<state_dim, control_dim>(N));

    problems.push_back(lqocProblem1);
    problems.push_back(lqocProblem2);

    std::shared_ptr<core::LinearSystem<state_dim, control_dim>> exampleSystem(
        new example::MIMOIntegratorLinear<state_dim, control_dim>());
    core::SensitivityApproximation<state_dim, control_dim> discreteExampleSystem(
        dt, exampleSystem, core::SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

    ct::core::ControlVector<control_dim> u0;
    u0.setZero();
    ct::core::StateVector<state_dim> x0;
    x0.setZero();
    ct::core::StateVector<state_dim> xf;
    xf.setRandom();

    auto costFunction = example::createMIMOIntegratorCostFunction<state_dim, control_dim>(xf);

    ct::core::StateVector<state_dim> b;
    b.setZero();


    for (size_t i = 0; i < lqocSolvers.size(); i++)
    {
        std::vector<double> setProblemTime;
        std::vector<double> allocTime;
        std::vector<double> solveTime;
        std::vector<double> getTime;
        std::vector<double> totalTime;

        size_t nRuns = 20000 / N;

        for (size_t j = 0; j < nRuns; j++)
        {
            problems[i]->setFromTimeInvariantLinearQuadraticProblem(discreteExampleSystem, *costFunction, b, dt);

            auto start_all = std::chrono::steady_clock::now();

            auto start_set = std::chrono::steady_clock::now();
            lqocSolvers[i]->setProblem(problems[i]);
            auto end_set = std::chrono::steady_clock::now();

            auto start_alloc = std::chrono::steady_clock::now();
            lqocSolvers[i]->initializeAndAllocate();
            auto end_alloc = std::chrono::steady_clock::now();

            auto start_solve = std::chrono::steady_clock::now();
            lqocSolvers[i]->solve();
            lqocSolvers[i]->computeStatesAndControls();
            lqocSolvers[i]->computeFeedbackMatrices();
            lqocSolvers[i]->compute_lv();
            auto end_solve = std::chrono::steady_clock::now();

            auto start_get = std::chrono::steady_clock::now();
            auto xSol = lqocSolvers[i]->getSolutionState();
            auto uSol = lqocSolvers[i]->getSolutionControl();
            auto end_get = std::chrono::steady_clock::now();

            auto end_all = std::chrono::steady_clock::now();

            // record times
            setProblemTime.push_back(std::chrono::duration<double, std::milli>(end_set - start_set).count());
            allocTime.push_back(std::chrono::duration<double, std::milli>(end_alloc - start_alloc).count());
            solveTime.push_back(std::chrono::duration<double, std::milli>(end_solve - start_solve).count());
            getTime.push_back(std::chrono::duration<double, std::milli>(end_get - start_get).count());
            totalTime.push_back(std::chrono::duration<double, std::milli>(end_all - start_all).count());
        }

        // average times
        double avgSetTime = std::accumulate(setProblemTime.begin(), setProblemTime.end(), 0.0) / (double)nRuns;
        double avgAllocTime = std::accumulate(allocTime.begin(), allocTime.end(), 0.0) / (double)nRuns;
        double avgSolveTime = std::accumulate(solveTime.begin(), solveTime.end(), 0.0) / (double)nRuns;
        double avgGetTime = std::accumulate(getTime.begin(), getTime.end(), 0.0) / (double)nRuns;
        double avgTotalTime = std::accumulate(totalTime.begin(), totalTime.end(), 0.0) / (double)nRuns;

        loggedSolveTimes[i].push_back(avgTotalTime);

        std::cout << "average setProblem() with " << solverNames[i] << " took\t" << avgSetTime << " ms" << std::endl;
        std::cout << "average initializeAndAllocate() with " << solverNames[i] << " took\t" << avgAllocTime << " ms"
                  << std::endl;
        std::cout << "average solve() with " << solverNames[i] << " took\t" << avgSolveTime << " ms" << std::endl;
        std::cout << "average get() with " << solverNames[i] << " took\t" << avgGetTime << " ms" << std::endl;
        std::cout << "average total call with " << solverNames[i] << " took\t" << avgTotalTime << " ms" << std::endl;

        std::cout << std::endl;
    }
    std::cout << std::endl;
}


template <size_t control_dim, size_t state_dim>
void timeSolvers()
{
    std::vector<size_t> testTimeHorizons = {5, 10, 50, 100, 500, 1000, 2000, 5000, 10000, 15000};

    std::cout << "Test System: " << std::endl;
    std::cout << "============ " << std::endl;
    std::cout << "state dim: " << state_dim << std::endl;
    std::cout << "control dim: " << control_dim << std::endl;

    std::vector<std::vector<double>> loggedSolveTimes;


    for (size_t i = 0; i < testTimeHorizons.size(); i++)
    {
        timeSingleSolve<control_dim, state_dim>(testTimeHorizons[i], loggedSolveTimes);
    }

    std::cout << std::endl << std::endl << std::endl << std::endl;

    std::cout << "Matlab friendly output:" << std::endl;
    std::cout << "stages = [";
    for (size_t i = 0; i < testTimeHorizons.size(); i++)
    {
        std::cout << testTimeHorizons[i];
        if (i < testTimeHorizons.size() - 1)
            std::cout << ", ";
    }
    std::cout << "];" << std::endl;

    for (size_t j = 0; j < 2; j++)
    {
        std::cout << "times_solver" << j << " = [";
        for (size_t i = 0; i < testTimeHorizons.size(); i++)
        {
            std::cout << loggedSolveTimes[j][i];
            if (i < testTimeHorizons.size() - 1)
                std::cout << ", ";
        }
        std::cout << "];" << std::endl;
    }
}

int main(int argc, char** argv)
{
    timeSolvers<3, 3>();
    timeSolvers<6, 6>();
    timeSolvers<12, 12>();
    timeSolvers<12, 24>();
    timeSolvers<12, 36>();

    return 1;
}
