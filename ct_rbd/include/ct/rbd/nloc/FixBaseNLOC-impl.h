/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::FixBaseNLOC(
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


template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::initialize(const tpl::JointState<NJOINTS, SCALAR>& x0,
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

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::initializeSteadyPose(
    const tpl::JointState<NJOINTS, SCALAR>& x0,
    const core::Time& tf,
    const int N,
	ControlVector& u_ref,
    FeedbackMatrix K)
{
    StateVector x0full = system_->toFullState(x0.toImplementation());


    ControlVector uff;
    computeIDTorques(x0, uff);  // compute inverse dynamics for this joint state
    u_ref = uff;

    // transcribe uff into the feed-forward init guess
    ControlVectorArray u0_ff(N, uff);

    // ... and to the act state if required // todo only works for this special case
    if (ACTUATOR_STATE_DIM > 0)
        x0full.template segment(NJOINTS, NJOINTS) = uff.template head<NJOINTS>();


    StateVectorArray x_ref = StateVectorArray(N + 1, x0full);
    FeedbackArray u0_fb(N, K);

    typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
    nlocSolver_->changeInitialState(x0full);
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::initializeDirectInterpolation(
    const tpl::JointState<NJOINTS, SCALAR>& x0,
    const tpl::JointState<NJOINTS, SCALAR>& xf,
    const core::Time& tf,
    const int N,
    FeedbackMatrix K)
{
    ControlVector uff;

    StateVectorArray x_ref(N + 1);
    ControlVectorArray u0_ff = ControlVectorArray(N);

    StateVector x0full = system_->toFullState(x0.toImplementation());
    StateVector xffull = system_->toFullState(xf.toImplementation());

    for (int i = 0; i < N + 1; i++)
    {
        x_ref[i] = x0full + (xffull - x0full) * SCALAR(i) / SCALAR(N);

        if (i < N)
        {
            typename RBDDynamics::RBDState_t rbdState = system_->RBDStateFromVector(x_ref[i]);
            const tpl::JointState<NJOINTS, SCALAR> jointState = rbdState.joints();
            computeIDTorques(jointState, u0_ff[i]);
        }
    }

    FeedbackArray u0_fb(N, K);

    typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

    nlocSolver_->changeInitialState(x0full);
    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
bool FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::runIteration()
{
    bool foundBetter = nlocSolver_->runIteration();

    iteration_++;
    return foundBetter;
}


template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
bool FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::solve()
{
    return nlocSolver_->solve();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const core::StateFeedbackController<FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::STATE_DIM,
    FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::CONTROL_DIM,
    SCALAR>&
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::getSolution()
{
    return nlocSolver_->getSolution();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::StateVectorArray&
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::retrieveLastRollout()
{
    return nlocSolver_->getStates();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const core::TimeArray& FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::getTimeArray()
{
    return nlocSolver_->getStateTrajectory().getTimeArray();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::FeedbackArray&
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::getFeedbackArray()
{
    return nlocSolver_->getSolution().K();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::ControlVectorArray&
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::getControlVectorArray()
{
    return nlocSolver_->getSolution().uff();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
const typename FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::NLOptConSolver::Settings_t&
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::getSettings() const
{
    return nlocSolver_->getSettings();
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::changeCostFunction(
    std::shared_ptr<CostFunction> costFunction)
{
    nlocSolver_->changeCostFunction(costFunction);
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
std::shared_ptr<typename FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::NLOptConSolver>
FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::getSolver()
{
    return nlocSolver_;
}

template <class RBDDynamics, size_t ACTUATOR_STATE_DIM, typename SCALAR>
void FixBaseNLOC<RBDDynamics, ACTUATOR_STATE_DIM, SCALAR>::computeIDTorques(const tpl::JointState<NJOINTS, SCALAR>& x,
    ControlVector& u)
{
    //! zero external link forces and acceleration
    typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());
    typename RBDDynamics::JointAcceleration_t jAcc(Eigen::Matrix<SCALAR, 6, 1>::Zero());

    system_->dynamics().FixBaseID(x, jAcc, linkForces, u);
}

}  // namespace rbd
}  // namespace ct
