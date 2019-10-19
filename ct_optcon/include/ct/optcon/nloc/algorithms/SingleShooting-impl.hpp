/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#include <ct/optcon/solver/NLOptConSettings.hpp>
#include <ct/optcon/nloc/NLOCAlgorithm.hpp>

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::SingleShooting(
    std::shared_ptr<Backend_t>& backend_,
    const Settings_t& settings)
    : Base(backend_)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::configure(const Settings_t& settings)
{
    this->backend_->configure(settings);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::setInitialGuess(
    const Policy_t& initialGuess)
{
    this->backend_->setInitialGuess(initialGuess);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::runIteration()
{
    prepareIteration();

    return finishIteration();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::prepareIteration()
{
    if (!this->backend_->isInitialized())
        throw std::runtime_error("SingleShooting is not initialized!");

    if (!this->backend_->isConfigured())
        throw std::runtime_error("SingleShooting is not configured!");

    this->backend_->checkProblem();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::finishIteration()
{
    int K = this->backend_->getNumSteps();

    bool debugPrint = this->backend_->getSettings().debugPrint;

    // if first iteration, compute shots and rollout and cost!
    if (this->backend_->iteration() == 0)
    {
        if (!this->backend_->nominalRollout())
            throw std::runtime_error("Rollout failed. System became unstable");

        this->backend_->updateCosts();
    }

#ifdef MATLAB_FULL_LOG
    if (this->backend_->iteration() == 0)
        this->backend_->logInitToMatlab();
#endif

    auto start = std::chrono::steady_clock::now();
    auto startEntire = start;

    // set box constraints and do LQ approximation
    this->backend_->setInputBoxConstraintsForLQOCProblem();
    this->backend_->setStateBoxConstraintsForLQOCProblem();
    this->backend_->computeLQApproximation(0, K - 1);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    if (debugPrint)
        std::cout << "[SingleShooting]: Computing LQ approximation took "
                  << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    end = std::chrono::steady_clock::now();
    diff = end - startEntire;
    if (debugPrint)
        std::cout << "[SingleShooting]: Forward pass took " << std::chrono::duration<double, std::milli>(diff).count()
                  << " ms" << std::endl;

    if (debugPrint)
        std::cout << "[SingleShooting]: #2 Solve LQOC Problem" << std::endl;

    start = std::chrono::steady_clock::now();
    this->backend_->solveFullLQProblem();
    this->backend_->extractSolution();
    end = std::chrono::steady_clock::now();
    diff = end - start;
    if (debugPrint)
        std::cout << "[SingleShooting]: Solving LQOC problem took "
                  << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    // update solutions and line-search
    if (debugPrint)
        std::cout << "[SingleShooting]: #3 LineSearch" << std::endl;

    start = std::chrono::steady_clock::now();
    bool foundBetter = this->backend_->lineSearch();
    end = std::chrono::steady_clock::now();
    diff = end - start;
    if (debugPrint)
        std::cout << "[SingleShooting]: Line search took " << std::chrono::duration<double, std::milli>(diff).count()
                  << " ms" << std::endl;

    diff = end - startEntire;
    if (debugPrint)
        std::cout << "[SingleShooting]: finishIteration took "
                  << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

    this->backend_->printSummary();

#ifdef MATLAB_FULL_LOG
    this->backend_->logToMatlab(this->backend_->iteration());
#endif

    this->backend_->iteration()++;

    return foundBetter;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::prepareMPCIteration()
{
    prepareIteration();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::finishMPCIteration()
{
    finishIteration();
    return true;  //! \todo : in MPC always returning true. Unclear how user wants to deal with varying costs, etc.
}

}  // namespace optcon
}  // namespace ct
