/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::NLOptConSolver(const OptConProblem_t& optConProblem,
    const Settings_t& settings)
{
    initialize(optConProblem, settings);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::NLOptConSolver(const OptConProblem_t& optConProblem,
    const std::string& settingsFile,
    bool verbose,
    const std::string& ns)
    : NLOptConSolver(optConProblem, Settings_t::fromConfigFile(settingsFile, verbose, ns))
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::~NLOptConSolver()
{
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::initialize(const OptConProblem_t& optConProblem,
    const Settings_t& settings)
{
    if (settings.nThreads > 1)
        nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>(
            new NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(optConProblem, settings));
    else
        nlocBackend_ = std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>(
            new NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(optConProblem, settings));

    configure(settings);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::configure(const Settings_t& settings)
{
    if (nlocBackend_->getSettings().nThreads != settings.nThreads)
        throw std::runtime_error("cannot switch from ST to MT or vice versa. Please call initialize.");

    nlocBackend_->configure(settings);

    switch (settings.nlocp_algorithm)
    {
        case NLOptConSettings::NLOCP_ALGORITHM::GNMS:
            nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>(
                new GNMS<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(nlocBackend_, settings));
            break;
        case NLOptConSettings::NLOCP_ALGORITHM::ILQR:
            nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>(
                new iLQR<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(nlocBackend_, settings));
            break;
        default:
            throw std::runtime_error("This algorithm is not implemented in NLOptConSolver.hpp");
    }
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::prepareIteration()
{
    nlocAlgorithm_->prepareIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::finishIteration()
{
    return nlocAlgorithm_->finishIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::prepareMPCIteration()
{
    nlocAlgorithm_->prepareMPCIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::finishMPCIteration()
{
    return nlocAlgorithm_->finishMPCIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::runIteration()
{
#ifdef DEBUG_PRINT
    auto startSolve = std::chrono::steady_clock::now();
#endif
    bool success = nlocAlgorithm_->runIteration();
#ifdef DEBUG_PRINT
    auto endSolve = std::chrono::steady_clock::now();
    std::cout << "[NLOC]: runIteration() took "
              << std::chrono::duration<double, std::milli>(endSolve - startSolve).count() << " ms" << std::endl;
#endif
    return success;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::setInitialGuess(const Policy_t& initialGuess)
{
    nlocBackend_->setInitialGuess(initialGuess);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::solve()
{
    bool foundBetter = true;
    int numIterations = 0;

    while (foundBetter && (numIterations < nlocBackend_->getSettings().max_iterations))
    {
        foundBetter = runIteration();

        numIterations++;
    }

    return (numIterations > 1 || foundBetter || (numIterations == 1 && !foundBetter));
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::Policy_t&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSolution()
{
    return nlocBackend_->getSolution();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::StateTrajectory<STATE_DIM, SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getStateTrajectory() const
{
    return nlocBackend_->getStateTrajectory();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::ControlTrajectory<CONTROL_DIM, SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getControlTrajectory() const
{
    return nlocBackend_->getControlTrajectory();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const core::tpl::TimeArray<SCALAR>& NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getTimeArray() const
{
    return nlocBackend_->getTimeArray();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getTimeHorizon() const
{
    return nlocBackend_->getTimeHorizon();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeTimeHorizon(const SCALAR& tf)
{
    nlocBackend_->changeTimeHorizon(tf);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeInitialState(
    const core::StateVector<STATE_DIM, SCALAR>& x0)
{
    nlocBackend_->changeInitialState(x0);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeCostFunction(
    const typename OptConProblem_t::CostFunctionPtr_t& cf)
{
    nlocBackend_->changeCostFunction(cf);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeNonlinearSystem(
    const typename OptConProblem_t::DynamicsPtr_t& dyn)
{
    nlocBackend_->changeNonlinearSystem(dyn);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::changeLinearSystem(
    const typename OptConProblem_t::LinearPtr_t& lin)
{
    nlocBackend_->changeLinearSystem(lin);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
SCALAR NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getCost() const
{
    return nlocBackend_->getCost();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::Settings_t&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getSettings()
{
    return nlocBackend_->getSettings();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::shared_ptr<NLOCBackendBase<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getBackend()
{
    return nlocBackend_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::DynamicsPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getNonlinearSystemsInstances()
{
    return nlocBackend_->getNonlinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::DynamicsPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getNonlinearSystemsInstances() const
{
    return nlocBackend_->getNonlinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::LinearPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getLinearSystemsInstances()
{
    return nlocBackend_->getLinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::LinearPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getLinearSystemsInstances() const
{
    return nlocBackend_->getLinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::CostFunctionPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getCostFunctionInstances()
{
    return nlocBackend_->getCostFunctionInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::CostFunctionPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getCostFunctionInstances() const
{
    return nlocBackend_->getCostFunctionInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getBoxConstraintsInstances()
{
    return nlocBackend_->getBoxConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getBoxConstraintsInstances() const
{
    return nlocBackend_->getBoxConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getGeneralConstraintsInstances()
{
    return nlocBackend_->getGeneralConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
const std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::OptConProblem_t::ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::getGeneralConstraintsInstances() const
{
    return nlocBackend_->getGeneralConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>::logSummaryToMatlab(const std::string& fileName)
{
    nlocBackend_->logSummaryToMatlab(fileName);
}

}  // namespace optcon
}  // namespace ct
