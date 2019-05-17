/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/systems/FloatingBaseFDSystem.h>
#include <ct/rbd/systems/linear/RbdLinearizer.h>

namespace ct {
namespace rbd {

/**
 * \brief NLOC for floating base systems with an explicit contact model.
 */
template <class RBDDynamics>
class FloatingBaseNLOCContactModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const bool quatIntegration = false;
    static const bool eeForcesAreControlInputs = false;

    typedef FloatingBaseFDSystem<RBDDynamics, false, false> FBSystem;
    typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> LinearizedSystem;
    typedef ct::rbd::RbdLinearizer<FBSystem> RBDLinearizer;
    typedef ct::core::SystemLinearizer<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> SystemLinearizer;

    typedef ct::optcon::NLOptConSolver<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> NLOptConSolver;

    typedef typename core::StateVector<FBSystem::STATE_DIM> StateVector;
    typedef typename core::ControlVector<FBSystem::CONTROL_DIM> ControlVector;
    typedef typename core::FeedbackMatrix<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> FeedbackMatrix;
    typedef typename core::StateVectorArray<FBSystem::STATE_DIM> StateVectorArray;
    typedef typename core::ControlVectorArray<FBSystem::CONTROL_DIM> ControlVectorArray;
    typedef typename core::FeedbackArray<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> FeedbackArray;

    typedef ct::optcon::CostFunctionAnalytical<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> CostFunction;


    //! constructor taking path to the settings file
    FloatingBaseNLOCContactModel(const std::string& costFunctionFile,
        const std::string& settingsFile,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);

    //! constructor taking settings file directly
    FloatingBaseNLOCContactModel(const std::string& costFunctionFile,
        const typename NLOptConSolver::Settings_t& settings,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);

    void initialize(const typename RBDDynamics::RBDState_t& x0,
        const core::Time& tf,
        StateVectorArray x_ref = StateVectorArray(),
        FeedbackArray u0_fb = FeedbackArray(),
        ControlVectorArray u0_ff = ControlVectorArray());

    void configure(const typename NLOptConSolver::Settings_t& settings);

    bool runIteration();

    const StateVectorArray& retrieveLastRollout();

    const StateVectorArray& getStateVectorArray();

    const core::TimeArray& getTimeArray();

    const FeedbackArray& getFeedbackArray();

    const ControlVectorArray& getControlVectorArray();

    const typename NLOptConSolver::Settings_t& getSettings() const;

    void changeCostFunction(std::shared_ptr<CostFunction> costFunction);

    std::shared_ptr<NLOptConSolver> getSolver();

private:
    std::shared_ptr<FBSystem> system_;
    std::shared_ptr<LinearizedSystem> linearizedSystem_;
    std::shared_ptr<CostFunction> costFunction_;

    optcon::ContinuousOptConProblem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> optConProblem_;

    std::shared_ptr<NLOptConSolver> solver_;

    size_t iteration_;
};
}
}
