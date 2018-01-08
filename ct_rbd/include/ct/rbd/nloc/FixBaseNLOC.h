/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/systems/FixBaseFDSystemSymplectic.h>

namespace ct {
namespace rbd {

/**
 * \brief NLOC for fixed base systems without an explicit contact model.
 */
template <class FIX_BASE_FD_SYSTEM, size_t ACTUATOR_STATE_DIM = 0, typename SCALAR = double>
class FixBaseNLOC
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const bool eeForcesAreControlInputs = false;

    using FBSystem = FIX_BASE_FD_SYSTEM;

    static const size_t CONTROL_DIM = FBSystem::CONTROL_DIM;
    static const size_t NJOINTS = FBSystem::NJOINTS;
    static const size_t STATE_DIM = FBSystem::STATE_DIM;

    typedef ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> LinearizedSystem;
    typedef ct::rbd::RbdLinearizer<FBSystem> SystemLinearizer;

    using JointAcceleration_t = ct::rbd::tpl::JointAcceleration<NJOINTS, SCALAR>;

    //! @ todo: introduce templates for P_DIM and V_DIM
    typedef ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR> NLOptConSolver;

    typedef typename core::StateVector<STATE_DIM, SCALAR> StateVector;
    typedef typename core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;
    typedef typename core::FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR> FeedbackMatrix;
    typedef typename core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
    typedef typename core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;
    typedef typename core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> FeedbackArray;
    typedef typename core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR> StateFeedbackController;

    typedef ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR> CostFunction;

    //! default constructor
    FixBaseNLOC() = default;

    //! constructor which directly takes a cost function
    FixBaseNLOC(std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>> costFun,
        const typename NLOptConSolver::Settings_t& nlocSettings,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        bool verbose = false,
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);

    void initialize(const tpl::JointState<NJOINTS, SCALAR>& x0,
        const core::Time& tf,
        StateVectorArray x_ref = StateVectorArray(),
        FeedbackArray u0_fb = FeedbackArray(),
        ControlVectorArray u0_ff = ControlVectorArray());

    //! initialize fixed-base robot with a steady pose using inverse dynamics torques as feedforward
    void initializeSteadyPose(const tpl::JointState<NJOINTS, SCALAR>& x0,
        const core::Time& tf,
        const int N,
        ControlVector& u_ref,
        FeedbackMatrix K = FeedbackMatrix::Zero());

    //! initialize fixed-base robot with a directly interpolated state trajectory and corresponding ID torques
    void initializeDirectInterpolation(const tpl::JointState<NJOINTS, SCALAR>& x0,
        const tpl::JointState<NJOINTS, SCALAR>& xf,
        const core::Time& tf,
        const int N,
        FeedbackMatrix K = FeedbackMatrix::Zero());

    //! initialize fixed-base robot with a directly interpolated state trajectory and corresponding ID torques
    void initializeDirectInterpolation(const tpl::JointState<NJOINTS, SCALAR>& x0,
        const tpl::JointState<NJOINTS, SCALAR>& xf,
        const core::Time& tf,
        const int N,
        ct::core::ControlVectorArray<NJOINTS, SCALAR>& u_array,
        ct::core::StateVectorArray<STATE_DIM, SCALAR>& x_array,
        FeedbackMatrix K = FeedbackMatrix::Zero());

    bool runIteration();

    bool solve();

    const core::StateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& getSolution();

    const StateVectorArray& retrieveLastRollout();

    const core::TimeArray& getTimeArray();

    const FeedbackArray& getFeedbackArray();

    const ControlVectorArray& getControlVectorArray();

    const typename NLOptConSolver::Settings_t& getSettings() const;

    void changeCostFunction(std::shared_ptr<CostFunction> costFunction);

    std::shared_ptr<NLOptConSolver> getSolver();

    //! compute fix-base inverse dynamics torques
    void computeIDTorques(const tpl::JointState<NJOINTS, SCALAR>& x, ControlVector& u);

private:
    std::shared_ptr<FBSystem> system_;
    std::shared_ptr<LinearizedSystem> linearizedSystem_;
    std::shared_ptr<CostFunction> costFunction_;

    optcon::OptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> optConProblem_;

    std::shared_ptr<NLOptConSolver> nlocSolver_;

    size_t iteration_;
};

}  // namespace rbd
}  // namespace ct
