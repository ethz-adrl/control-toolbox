/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#include <ct/rbd/systems/FixBaseFDSystem.h>

namespace ct {
namespace rbd {

/**
 * \brief NLOC for fixed base systems without an explicit contact model.
 */
template <class RBDDynamics, typename SCALAR = double>
class FixBaseNLOC
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const bool eeForcesAreControlInputs = false;

    typedef FixBaseFDSystem<RBDDynamics, eeForcesAreControlInputs> FBSystem;
    typedef ct::core::LinearSystem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, SCALAR> LinearizedSystem;
    typedef ct::rbd::RbdLinearizer<FBSystem> SystemLinearizer;

    typedef typename RBDDynamics::JointAcceleration_t JointAcceleration_t;
    typedef typename RBDDynamics::ExtLinkForces_t ExtLinkForces_t;

    //! @ todo: introduce templates for P_DIM and V_DIM
    typedef ct::optcon::NLOptConSolver<FBSystem::STATE_DIM,
        FBSystem::CONTROL_DIM,
        FBSystem::STATE_DIM / 2,
        FBSystem::STATE_DIM / 2,
        SCALAR>
        NLOptConSolver;

    typedef typename core::StateVector<FBSystem::STATE_DIM, SCALAR> StateVector;
    typedef typename core::ControlVector<FBSystem::CONTROL_DIM, SCALAR> ControlVector;
    typedef typename core::FeedbackMatrix<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, SCALAR> FeedbackMatrix;
    typedef typename core::StateVectorArray<FBSystem::STATE_DIM, SCALAR> StateVectorArray;
    typedef typename core::ControlVectorArray<FBSystem::CONTROL_DIM, SCALAR> ControlVectorArray;
    typedef typename core::FeedbackArray<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, SCALAR> FeedbackArray;
    typedef typename core::StateFeedbackController<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, SCALAR>
        StateFeedbackController;

    typedef ct::optcon::CostFunctionAD<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM> CostFunction;


    //! constructor for loading nloc settings from file
    FixBaseNLOC(const std::string& costFunctionFile,
        const std::string& settingsFile,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        bool verbose = false,
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);


    //! constructor which directly takes the nloc settings
    FixBaseNLOC(const std::string& costFunctionFile,
        const typename NLOptConSolver::Settings_t& nlocSettings,
        std::shared_ptr<FBSystem> system = std::shared_ptr<FBSystem>(new FBSystem),
        bool verbose = false,
        std::shared_ptr<LinearizedSystem> linearizedSystem = nullptr);


    void initialize(const tpl::JointState<FBSystem::CONTROL_DIM, SCALAR>& x0,
        const core::Time& tf,
        StateVectorArray x_ref = StateVectorArray(),
        FeedbackArray u0_fb = FeedbackArray(),
        ControlVectorArray u0_ff = ControlVectorArray());


    //! initialize fixed-base robot with a steady pose using inverse dynamics torques as feedforward
    void initializeSteadyPose(const tpl::JointState<FBSystem::CONTROL_DIM, SCALAR>& x0,
        const core::Time& tf,
        const int N,
        FeedbackMatrix K = FeedbackMatrix::Zero());


    //! initialize fixed-base robot with a directly interpolated state trajectory and corresponding ID torques
    void initializeDirectInterpolation(const tpl::JointState<FBSystem::CONTROL_DIM, SCALAR>& x0,
        const tpl::JointState<FBSystem::CONTROL_DIM, SCALAR>& xf,
        const core::Time& tf,
        const int N,
        FeedbackMatrix K = FeedbackMatrix::Zero());


    bool runIteration();

    const core::StateFeedbackController<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, SCALAR>& getSolution();

    const StateVectorArray& retrieveLastRollout();

    const core::TimeArray& getTimeArray();

    const FeedbackArray& getFeedbackArray();

    const ControlVectorArray& getControlVectorArray();

    const typename NLOptConSolver::Settings_t& getSettings() const;

    void changeCostFunction(std::shared_ptr<CostFunction> costFunction);

    std::shared_ptr<NLOptConSolver> getSolver();


private:
    //! compute fix-base inverse dynamics torques for initialization
    void computeIDTorques(const tpl::JointState<FBSystem::CONTROL_DIM, SCALAR>& x, ControlVector& u);

    std::shared_ptr<FBSystem> system_;
    std::shared_ptr<LinearizedSystem> linearizedSystem_;
    std::shared_ptr<CostFunction> costFunction_;

    optcon::OptConProblem<FBSystem::STATE_DIM, FBSystem::CONTROL_DIM, SCALAR> optConProblem_;

    std::shared_ptr<NLOptConSolver> nlocSolver_;

    size_t iteration_;
};

}  // namespace rbd
}  // namespace ct
