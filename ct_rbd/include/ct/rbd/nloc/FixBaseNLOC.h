/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/systems/FixBaseFDSystemSymplectic.h>

namespace ct {
namespace rbd {

/**
 * \brief NLOC for fixed base systems without an explicit contact model.
 */
template <class FIX_BASE_FD_SYSTEM>
class FixBaseNLOC
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const bool eeForcesAreControlInputs = false;

    using FBSystem = FIX_BASE_FD_SYSTEM;

    static const size_t CONTROL_DIM = FBSystem::CONTROL_DIM;
    static const size_t NJOINTS = FBSystem::NJOINTS;
    static const size_t STATE_DIM = FBSystem::STATE_DIM;
    static const size_t ACTUATOR_STATE_DIM = FBSystem::ACTUATOR_STATE_DIM;
    using SCALAR = typename FBSystem::SCALAR;

    using LinearizedSystem = ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>;
    //    using SystemLinearizer = ct::rbd::RbdLinearizer<FBSystem>;

    //! @ todo: introduce templates for P_DIM and V_DIM
    using NLOptConSolver = ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>;

    using RobotState_t = FixBaseRobotState<NJOINTS, ACTUATOR_STATE_DIM, SCALAR>;
    using StateVector = typename core::StateVector<STATE_DIM, SCALAR>;
    using ControlVector = typename core::ControlVector<CONTROL_DIM, SCALAR>;
    using FeedbackMatrix = typename core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>;
    using StateVectorArray = typename core::StateVectorArray<STATE_DIM, SCALAR>;
    using ControlVectorArray = typename core::ControlVectorArray<CONTROL_DIM, SCALAR>;
    using FeedbackArray = typename core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>;
    using StateFeedbackController = typename core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>;

    using CostFunction = ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>;
    using JointAcceleration_t = JointAcceleration<NJOINTS, SCALAR>;


    //! default constructor
    FixBaseNLOC() = default;

    //! constructor which directly takes a cost function
    FixBaseNLOC(std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFun,
        const typename NLOptConSolver::Settings_t& nlocSettings,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        bool verbose = false,
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);

    //! constructor which directly takes a cost function and constraints, mind the order of the constraints
    FixBaseNLOC(std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFun,
        std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> inputBoxConstraints,
        std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> stateBoxConstraints,
        std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> generalConstraints,
        const typename NLOptConSolver::Settings_t& nlocSettings,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        bool verbose = false,
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);

    void initialize(const RobotState_t& x0,
        const core::Time& tf,
        StateVectorArray x_ref = StateVectorArray(),
        FeedbackArray u0_fb = FeedbackArray(),
        ControlVectorArray u0_ff = ControlVectorArray());

    //! initialize fixed-base robot with a steady pose using inverse dynamics torques as feedforward
    void initializeSteadyPose(const RobotState_t& x0,
        const core::Time& tf,
        const int N,
        ControlVector& u_ref,
        FeedbackMatrix K = FeedbackMatrix::Zero());

    //! initialize fixed-base robot with a directly interpolated state trajectory and corresponding ID torques
    void initializeDirectInterpolation(const RobotState_t& x0,
        const RobotState_t& xf,
        const core::Time& tf,
        const int N,
        FeedbackMatrix K = FeedbackMatrix::Zero());

    //! initialize fixed-base robot with a directly interpolated state trajectory and corresponding ID torques
    void initializeDirectInterpolation(const RobotState_t& x0,
        const RobotState_t& xf,
        const core::Time& tf,
        const int N,
        ct::core::ControlVectorArray<NJOINTS, SCALAR>& u_array,
        ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_array,
        FeedbackMatrix K = FeedbackMatrix::Zero());

    bool runIteration();

    bool solve();

    const StateFeedbackController& getSolution();

    const core::TimeArray& getTimeArray();

    const FeedbackArray& getFeedbackArray();

    const ControlVectorArray& getControlVectorArray();

    const StateVectorArray& getStateVectorArray();

    const typename NLOptConSolver::Settings_t& getSettings() const;

    void changeCostFunction(std::shared_ptr<CostFunction> costFunction);

    std::shared_ptr<NLOptConSolver> getSolver();

private:
    std::shared_ptr<FBSystem> system_;
    std::shared_ptr<LinearizedSystem> linearizedSystem_;
    std::shared_ptr<CostFunction> costFunction_;
    std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> inputBoxConstraints_;
    std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> stateBoxConstraints_;
    std::shared_ptr<ct::optcon::LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>> generalConstraints_;

    optcon::ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> optConProblem_;

    std::shared_ptr<NLOptConSolver> nlocSolver_;

    size_t iteration_;
};

}  // namespace rbd
}  // namespace ct
