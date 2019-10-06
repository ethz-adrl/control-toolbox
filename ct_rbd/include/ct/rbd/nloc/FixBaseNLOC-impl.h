/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <class FIX_BASE_FD_SYSTEM>
FixBaseNLOC<FIX_BASE_FD_SYSTEM>::FixBaseNLOC(
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

template <class FIX_BASE_FD_SYSTEM>
FixBaseNLOC<FIX_BASE_FD_SYSTEM>::FixBaseNLOC(
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFun,
    std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> inputBoxConstraints,
    std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> stateBoxConstraints,
    std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> generalConstraints,
    const typename NLOptConSolver::Settings_t& nlocSettings,
    std::shared_ptr<FBSystem> system,
    bool verbose,
    std::shared_ptr<LinearizedSystem> linearizedSystem)
    : system_(system),
      linearizedSystem_(linearizedSystem),
      inputBoxConstraints_(inputBoxConstraints),
      stateBoxConstraints_(stateBoxConstraints),
      generalConstraints_(generalConstraints),
      costFunction_(costFun),
      optConProblem_(system_, costFunction_, linearizedSystem_),
      iteration_(0)
{
    if (inputBoxConstraints_ != nullptr)
        optConProblem_.setInputBoxConstraints(inputBoxConstraints_);
    if (stateBoxConstraints_ != nullptr)
        optConProblem_.setStateBoxConstraints(stateBoxConstraints_);
    if (generalConstraints_ != nullptr)
        optConProblem_.setGeneralConstraints(generalConstraints_);

    optConProblem_.verify();
    nlocSolver_ = std::shared_ptr<NLOptConSolver>(new NLOptConSolver(optConProblem_, nlocSettings));
}

template <class FIX_BASE_FD_SYSTEM>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM>::initialize(const RobotState_t& x0,
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

template <class FIX_BASE_FD_SYSTEM>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM>::initializeSteadyPose(const RobotState_t& x0,
    const core::Time& tf,
    const int N,
    ControlVector& u_ref,
    FeedbackMatrix K)
{
    ControlVector uff_torque = system_->computeIDTorques(x0.joints());

    if (ACTUATOR_STATE_DIM > 0)  // if there are actuator dynamics
    {
        u_ref.setZero();  // TODO compute this from act.state
    }
    else  // actuator dynamics off
    {
        u_ref = uff_torque;
    }

    // transcribe uff into the feed-forward init guess
    ControlVectorArray u0_ff(N, u_ref);
    StateVectorArray x_ref = StateVectorArray(N + 1, x0.toStateVector());
    FeedbackArray u0_fb(N, K);
    typename NLOptConSolver::Policy_t policy(x_ref, u0_ff, u0_fb, getSettings().dt);

    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
    nlocSolver_->changeInitialState(x_ref.front());
}

template <class FIX_BASE_FD_SYSTEM>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM>::initializeDirectInterpolation(const RobotState_t& x0,
    const RobotState_t& xf,
    const core::Time& tf,
    const int N,
    FeedbackMatrix K)
{
    ct::core::ControlVectorArray<NJOINTS, SCALAR> uff_array;
    ct::core::StateVectorArray<STATE_DIM, SCALAR> x_array;

    initializeDirectInterpolation(x0, xf, tf, N, uff_array, x_array, K);
}


template <class FIX_BASE_FD_SYSTEM>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM>::initializeDirectInterpolation(const RobotState_t& x0,
    const RobotState_t& xf,
    const core::Time& tf,
    const int N,
    ct::core::ControlVectorArray<NJOINTS, SCALAR>& uff_array,
    ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_array,
    FeedbackMatrix K)
{
    uff_array.resize(N);
    x_array.resize(N + 1);

    // temporary variables
    ControlVector uff_torque;

    for (int i = 0; i < N + 1; i++)
    {
        RobotState_t state_temp = x0;
        state_temp.joints().toImplementation() =
            x0.joints().toImplementation() +
            (xf.joints().toImplementation() - x0.joints().toImplementation()) * SCALAR(i) / SCALAR(N);

        if (i < N)
        {
            uff_torque = system_->computeIDTorques(state_temp.joints());
            if (ACTUATOR_STATE_DIM > 0)  // if there are actuator dynamics
            {
                uff_array[i].setZero();  // todo compute this from act.state
            }
            else  // actuator dynamics off
            {
                uff_array[i] = uff_torque;
            }
        }

        if (i > 0 && ACTUATOR_STATE_DIM > 0)
        {
            // direct interpolation for actuator state may not make sense. Improve using actuator model
            state_temp.actuatorState() =
                system_->getActuatorDynamics()->computeStateFromOutput(state_temp.joints(), uff_torque);
        }

        x_array[i] = state_temp.toStateVector();
    }

    FeedbackArray u0_fb(N, K);

    typename NLOptConSolver::Policy_t policy(x_array, uff_array, u0_fb, getSettings().dt);

    std::cout << "init with " << x0.toStateVector() << std::endl;

    nlocSolver_->changeInitialState(x0.toStateVector());
    nlocSolver_->changeTimeHorizon(tf);
    nlocSolver_->setInitialGuess(policy);
}

template <class FIX_BASE_FD_SYSTEM>
bool FixBaseNLOC<FIX_BASE_FD_SYSTEM>::runIteration()
{
    bool foundBetter = nlocSolver_->runIteration();

    iteration_++;
    return foundBetter;
}


template <class FIX_BASE_FD_SYSTEM>
bool FixBaseNLOC<FIX_BASE_FD_SYSTEM>::solve()
{
    return nlocSolver_->solve();
}

template <class FIX_BASE_FD_SYSTEM>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM>::StateFeedbackController& FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getSolution()
{
    return nlocSolver_->getSolution();
}

template <class FIX_BASE_FD_SYSTEM>
const core::TimeArray& FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getTimeArray()
{
    return nlocSolver_->getStateTrajectory().getTimeArray();
}

template <class FIX_BASE_FD_SYSTEM>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM>::FeedbackArray& FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getFeedbackArray()
{
    return nlocSolver_->getSolution().K();
}

template <class FIX_BASE_FD_SYSTEM>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM>::ControlVectorArray&
FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getControlVectorArray()
{
    return nlocSolver_->getSolution().uff();
}

template <class FIX_BASE_FD_SYSTEM>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM>::StateVectorArray& FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getStateVectorArray()
{
    return nlocSolver_->getSolution().x_ref();
}

template <class FIX_BASE_FD_SYSTEM>
const typename FixBaseNLOC<FIX_BASE_FD_SYSTEM>::NLOptConSolver::Settings_t&
FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getSettings() const
{
    return nlocSolver_->getSettings();
}

template <class FIX_BASE_FD_SYSTEM>
void FixBaseNLOC<FIX_BASE_FD_SYSTEM>::changeCostFunction(std::shared_ptr<CostFunction> costFunction)
{
    nlocSolver_->changeCostFunction(costFunction);
}

template <class FIX_BASE_FD_SYSTEM>
std::shared_ptr<typename FixBaseNLOC<FIX_BASE_FD_SYSTEM>::NLOptConSolver> FixBaseNLOC<FIX_BASE_FD_SYSTEM>::getSolver()
{
    return nlocSolver_;
}

}  // namespace rbd
}  // namespace ct
