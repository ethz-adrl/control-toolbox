/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::FixBaseNLOC(
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFun,
    const typename NLOptConSolver::Settings_t& nlocSettings,
    std::shared_ptr<FBSystem> system,
    bool verbose,
    std::shared_ptr<LinearizedSystem> linearizedSystem)
    : system_(system),
      linearizedSystem_(linearizedSystem),
      costFunction_(costFun),
      optConProblem_(system_, costFunction_, linearizedSystem_),
      iteration_(0)
{
    optConProblem_.verify();
    nlocSolver_ = std::shared_ptr<NLOptConSolver>(new NLOptConSolver(optConProblem_, nlocSettings));
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::initialize(const tpl::JointState<NJOINTS, SCALAR>& x0,
    const core::Time& tf,
    StateVectorArray x_ref,
    FeedbackArray u0_fb,
    ControlVectorArray u0_ff)
{
    typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
    nlocSolver_->changeInitialState(x0.toImplementation());
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::initializeSteadyPose(
    const tpl::JointState<NJOINTS, SCALAR>& x0,
    const core::Time& tf,
    const int N,
    ControlVector& u_ref,
    FeedbackMatrix K)
{
    ct::core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actRefState;
    actRefState.setZero();

    ControlVector uff;
    computeIDTorques(x0, uff);  // compute inverse dynamics for this joint state
    u_ref = uff;

    // transcribe uff into the feed-forward init guess
    ControlVectorArray u0_ff(N, uff);

    // ... and to the act state if required // todo only works for this special case
    if (ACTUATOR_STATE_DIM > 0)
    {
        actRefState = system_->getActuatorDynamics()->computeStateFromOutput(x0, uff);
    }

    StateVector x0full = system_->toFullState(x0.toImplementation(), actRefState);

    StateVectorArray x_ref = StateVectorArray(N + 1, x0full);
    FeedbackArray u0_fb(N, K);

    typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
    nlocSolver_->changeInitialState(x0full);
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::initializeDirectInterpolation(
    const tpl::JointState<NJOINTS, SCALAR>& x0,
    const tpl::JointState<NJOINTS, SCALAR>& xf,
    const core::Time& tf,
    const int N,
    FeedbackMatrix K)
{
    ct::core::ControlVectorArray<NJOINTS, SCALAR> uff_array;
    ct::core::StateVectorArray<STATE_DIM, SCALAR> x_array;

    initializeDirectInterpolation(x0, xf, tf, N, uff_array, x_array, K);
}


template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::initializeDirectInterpolation(
    const tpl::JointState<NJOINTS, SCALAR>& x0,
    const tpl::JointState<NJOINTS, SCALAR>& xf,
    const core::Time& tf,
    const int N,
    ct::core::ControlVectorArray<NJOINTS, SCALAR>& uff_array,
    ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_array,
    FeedbackMatrix K)
{
    uff_array.resize(N);
    x_array.resize(N + 1);

    ControlVector uff;

    StateVector x0full = system_->toFullState(x0.toImplementation());
    StateVector xffull = system_->toFullState(xf.toImplementation());

    for (int i = 0; i < N + 1; i++)
    {
        x_array[i] = x0full + (xffull - x0full) * SCALAR(i) / SCALAR(N);

        if (i < N)
        {
            ct::rbd::tpl::RBDState<NJOINTS, SCALAR> rbdState = system_->RBDStateFromVector(x_array[i]);
            const tpl::JointState<NJOINTS, SCALAR> jointState = rbdState.joints();
            computeIDTorques(jointState, uff_array[i]);
        }
    }

    FeedbackArray u0_fb(N, K);

    typename NLOptConSolver::Policy_t policy(x_array, uff_array, u0_fb, getSettings().dt);

    nlocSolver_->changeInitialState(x0full);
    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
bool FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::runIteration()
{
    bool foundBetter = nlocSolver_->runIteration();

    iteration_++;
    return foundBetter;
}


template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
bool FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::solve()
{
    return nlocSolver_->solve();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const core::StateFeedbackController<FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::STATE_DIM,
    FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::CONTROL_DIM,
    SCALAR>&
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::getSolution()
{
    return nlocSolver_->getSolution();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::StateVectorArray&
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::retrieveLastRollout()
{
    return nlocSolver_->getStates();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const core::TimeArray& FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::getTimeArray()
{
    return nlocSolver_->getStateTrajectory().getTimeArray();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::FeedbackArray&
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::getFeedbackArray()
{
    return nlocSolver_->getSolution().K();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::ControlVectorArray&
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::getControlVectorArray()
{
    return nlocSolver_->getSolution().uff();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::NLOptConSolver::Settings_t&
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::getSettings() const
{
    return nlocSolver_->getSettings();
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::changeCostFunction(
    std::shared_ptr<CostFunction> costFunction)
{
    nlocSolver_->changeCostFunction(costFunction);
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
std::shared_ptr<typename FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::NLOptConSolver>
FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::getSolver()
{
    return nlocSolver_;
}

template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM, ACTUATOR_STATE_DIM, SCALAR>::computeIDTorques(
    const tpl::JointState<NJOINTS, SCALAR>& x,
    ControlVector& u)
{
    //! zero external link forces and acceleration
    JointAcceleration_t jAcc(Eigen::Matrix<SCALAR, NJOINTS, 1>::Zero());

    system_->dynamics().FixBaseID(x, jAcc, u);
}

}  // namespace rbd
}  // namespace ct
