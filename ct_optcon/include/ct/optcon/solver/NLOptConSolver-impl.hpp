/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::NLOptConSolver(
    const OptConProblem_t& optConProblem,
    const Settings_t& settings)
{
    initialize(optConProblem, settings);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::NLOptConSolver(
    const OptConProblem_t& optConProblem,
    const std::string& settingsFile,
    bool verbose,
    const std::string& ns)
    : NLOptConSolver(optConProblem, Settings_t::fromConfigFile(settingsFile, verbose, ns))
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::setAlgorithm(const Settings_t& settings)
{
    if (settings.isSingleShooting())
    {
        nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>>(
            new SingleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>(nlocBackend_, settings));
    }
    else
    {
        nlocAlgorithm_ = std::shared_ptr<NLOCAlgorithm<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>>(
            new MultipleShooting<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>(nlocBackend_, settings));
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::initialize(
    const OptConProblem_t& optConProblem,
    const Settings_t& settings)
{
    if (settings.nThreads > 1)
        nlocBackend_ = std::shared_ptr<Backend_t>(
            new NLOCBackendMP<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>(optConProblem, settings));
    else
        nlocBackend_ = std::shared_ptr<Backend_t>(
            new NLOCBackendST<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>(optConProblem, settings));

    setAlgorithm(settings);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::configure(const Settings_t& settings)
{
    if (nlocBackend_->getSettings().nThreads != settings.nThreads)
        throw std::runtime_error("cannot switch from ST to MT or vice versa. Please call initialize.");

    nlocBackend_->configure(settings);

    setAlgorithm(settings);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::prepareIteration()
{
    nlocAlgorithm_->prepareIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::finishIteration()
{
    return nlocAlgorithm_->finishIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::prepareMPCIteration()
{
    nlocAlgorithm_->prepareMPCIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::finishMPCIteration()
{
    return nlocAlgorithm_->finishMPCIteration();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::runIteration()
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


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::setInitialGuess(
    const Policy_t& initialGuess)
{
    nlocBackend_->setInitialGuess(initialGuess);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
bool NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::solve()
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


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::Policy_t&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getSolution()
{
    return nlocBackend_->getSolution();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const core::StateTrajectory<STATE_DIM, SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getStateTrajectory() const
{
    return nlocBackend_->getStateTrajectory();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const core::ControlTrajectory<CONTROL_DIM, SCALAR>
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getControlTrajectory() const
{
    return nlocBackend_->getControlTrajectory();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const core::tpl::TimeArray<SCALAR>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getTimeArray() const
{
    return nlocBackend_->getTimeArray();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
SCALAR NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getTimeHorizon() const
{
    return nlocBackend_->getTimeHorizon();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::changeTimeHorizon(const SCALAR& tf)
{
    nlocBackend_->changeTimeHorizon(tf);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::changeInitialState(
    const core::StateVector<STATE_DIM, SCALAR>& x0)
{
    nlocBackend_->changeInitialState(x0);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::changeCostFunction(
    const typename OptConProblem_t::CostFunctionPtr_t& cf)
{
    nlocBackend_->changeCostFunction(cf);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::changeNonlinearSystem(
    const typename OptConProblem_t::DynamicsPtr_t& dyn)
{
    nlocBackend_->changeNonlinearSystem(dyn);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::changeLinearSystem(
    const typename OptConProblem_t::LinearPtr_t& lin)
{
    nlocBackend_->changeLinearSystem(lin);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
SCALAR NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getCost() const
{
    return nlocBackend_->getCost();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::Settings_t&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getSettings()
{
    return nlocBackend_->getSettings();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::shared_ptr<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::Backend_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getBackend()
{
    return nlocBackend_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::DynamicsPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getNonlinearSystemsInstances()
{
    return nlocBackend_->getNonlinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::DynamicsPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getNonlinearSystemsInstances() const
{
    return nlocBackend_->getNonlinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::LinearPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getLinearSystemsInstances()
{
    return nlocBackend_->getLinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::vector<
    typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::LinearPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getLinearSystemsInstances() const
{
    return nlocBackend_->getLinearSystemsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        CostFunctionPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getCostFunctionInstances()
{
    return nlocBackend_->getCostFunctionInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        CostFunctionPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getCostFunctionInstances() const
{
    return nlocBackend_->getCostFunctionInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getInputBoxConstraintsInstances()
{
    return nlocBackend_->getInputBoxConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getInputBoxConstraintsInstances() const
{
    return nlocBackend_->getInputBoxConstraintsInstances();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getStateBoxConstraintsInstances()
{
    return nlocBackend_->getStateBoxConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getStateBoxConstraintsInstances() const
{
    return nlocBackend_->getStateBoxConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getGeneralConstraintsInstances()
{
    return nlocBackend_->getGeneralConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
const std::vector<typename NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::OptConProblem_t::
        ConstraintPtr_t>&
NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::getGeneralConstraintsInstances() const
{
    return nlocBackend_->getGeneralConstraintsInstances();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM, typename SCALAR, bool CONTINUOUS>
void NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR, CONTINUOUS>::logSummaryToMatlab(
    const std::string& fileName)
{
    nlocBackend_->logSummaryToMatlab(fileName);
}

}  // namespace optcon
}  // namespace ct
